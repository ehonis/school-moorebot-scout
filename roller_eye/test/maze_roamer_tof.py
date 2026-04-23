import signal
import select
import sys
import threading
import time
import tty
import termios

import rospy
from sensor_msgs.msg import Range

try:
    # Nav services are available on the robot ROS image.
    # `nav_patrol(name="")` is interpreted by NavPath as "go back home".
    from roller_eye.srv import nav_cancel, nav_patrol
except Exception:
    # Keep script runnable even if service stubs are unavailable in this env.
    nav_patrol = None
    nav_cancel = None


# The Scout Python API is typically installed at this path on-device.
sys.path.append("/usr/local/lib")
from rollereye import *  # noqa: F401,F403 - project examples use star import


# =========================
# Tuning parameters
# =========================
# Stop when something is very close in front (about 5-10 cm).
# Set to 8 cm as the default "wall too close" trigger.
STOP_DISTANCE_M = 0.08

# Use a little hysteresis to avoid rapid stop/start jitter near the threshold.
# Robot resumes forward when clearance grows beyond this distance.
RESUME_DISTANCE_M = 0.12

# Continuous forward speed (m/s).
FORWARD_SPEED_MPS = 0.16

# Rotation speed used for scan turns (deg/s).
TURN_SPEED_DPS = 90

# Scan resolution. Smaller step = better heading search, slower scan.
SCAN_STEP_DEG = 30

# Do not pick headings in the rear half of the robot.
# This keeps recovery turns from backtracking toward already-traveled space.
# Interpretation:
#   - 0 deg offset is straight ahead (always allowed)
#   - +/-90 deg is still considered forward-sector
#   - anything beyond that (rear hemisphere) is disallowed
MAX_FORWARD_TURN_DEG = 90

# If TOF data is stale, pause movement for safety.
TOF_STALE_TIMEOUT_S = 1.0

# Timeout to wait for one fresh TOF reading after each turn.
TOF_WAIT_TIMEOUT_S = 0.5

# Tiny settle wait to let motion/reading stabilize between operations.
SETTLE_S = 0.15

# Periodic TOF telemetry logging interval (seconds).
# Lower this if you want denser logs.
TOF_LOG_INTERVAL_S = 0.25

# Manual override speed values (kept modest for tight spaces).
MANUAL_FORWARD_SPEED_MPS = 0.16
MANUAL_TURN_SPEED_DPS = 75

# Service names provided by NavPath node.
NAV_PATROL_SERVICE = "/nav_patrol"
NAV_CANCEL_SERVICE = "/nav_cancel"


# Shared TOF snapshot (callback thread -> control loop thread).
_tof_lock = threading.Lock()
_latest_tof_range_m = None
_latest_tof_time_s = 0.0

# Main loop run flag; set False by signal handler.
_running = True


class KeyboardReader(object):
    """
    Read single keystrokes without blocking the control loop.

    We switch stdin to cbreak mode so each key is available immediately
    (no Enter required), then restore terminal settings on shutdown.
    """

    def __init__(self):
        self._enabled = False
        self._fd = None
        self._old_settings = None

    def setup(self):
        """Enable non-blocking single-key reads if stdin is a real TTY."""
        if not sys.stdin.isatty():
            print("Keyboard override disabled (stdin is not a TTY).")
            return

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._enabled = True

    def read_key(self):
        """
        Return one pressed key or None if no key is waiting.
        """
        if not self._enabled:
            return None

        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            return sys.stdin.read(1)
        return None

    def restore(self):
        """Restore original terminal settings."""
        if self._enabled and self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        self._enabled = False


def tof_callback(msg):
    """Save the latest TOF range and timestamp from ROS callback."""
    global _latest_tof_range_m, _latest_tof_time_s
    with _tof_lock:
        _latest_tof_range_m = msg.range
        _latest_tof_time_s = time.time()


def get_tof_snapshot():
    """Return (range_m, timestamp_s) from the latest TOF callback."""
    with _tof_lock:
        return _latest_tof_range_m, _latest_tof_time_s


def wait_for_fresh_tof(max_wait_s):
    """
    Wait briefly for a fresh TOF value and return it.

    Returns:
      float distance in meters, or None if no fresh value arrives in time.
    """
    start = time.time()
    while time.time() - start <= max_wait_s:
        range_m, ts = get_tof_snapshot()
        if range_m is not None and (time.time() - ts) <= TOF_STALE_TIMEOUT_S:
            return range_m
        time.sleep(0.01)
    return None


def rotate_deg(direction, degrees):
    """
    Rotate by exact degrees using Scout API.

    direction:
      1 = left (counter-clockwise)
      2 = right (clockwise)
    """
    rollereye.set_rotationSpeed(TURN_SPEED_DPS)
    rollereye.set_rotate_3(direction=direction, degree=degrees)
    time.sleep(SETTLE_S)


def choose_best_heading_with_scan():
    """
    Scan 360 degrees in SCAN_STEP_DEG increments and choose best heading.

    We rotate right one step at a time, sample front TOF after each step,
    then finish the full circle (ending back at the original heading).

    Returns:
      best_offset_right_deg: heading offset (from original), turning right.
      best_distance_m: measured clearance at that heading.
    """
    steps = int(360 / SCAN_STEP_DEG)

    # Measure current heading first (offset = 0).
    best_distance = wait_for_fresh_tof(TOF_WAIT_TIMEOUT_S)
    if best_distance is None:
        best_distance = 0.0
    best_offset_right_deg = 0

    # Rotate through all headings and track best clearance.
    # Even though we measure all 360 degrees, we intentionally ignore rear
    # hemisphere candidates so the robot does not choose a backtracking route.
    for step_idx in range(1, steps):
        rotate_deg(direction=2, degrees=SCAN_STEP_DEG)
        d = wait_for_fresh_tof(TOF_WAIT_TIMEOUT_S)
        if d is None:
            d = 0.0

        offset = step_idx * SCAN_STEP_DEG
        if is_offset_in_forward_sector(offset) and d > best_distance:
            best_distance = d
            best_offset_right_deg = offset

    # Final step to return to original heading (completes full 360 scan).
    rotate_deg(direction=2, degrees=SCAN_STEP_DEG)

    return best_offset_right_deg, best_distance


def is_offset_in_forward_sector(offset_right_deg):
    """
    Return True if a heading offset stays in the allowed forward sector.

    The scan offsets are measured as "right turn from current heading":
      - 0   : straight ahead
      - 90  : right side
      - 180 : straight behind
      - 270 : left side

    We convert that to smallest absolute turn from forward and allow only
    offsets up to MAX_FORWARD_TURN_DEG. This excludes the rear hemisphere,
    which prevents selecting "go backward/backtrack" escape headings.
    """
    offset = offset_right_deg % 360
    smallest_turn_from_forward = min(offset, 360 - offset)
    return smallest_turn_from_forward <= MAX_FORWARD_TURN_DEG


def turn_to_best_heading(best_offset_right_deg):
    """
    Turn from current heading to the selected best heading with shortest turn.
    """
    if best_offset_right_deg == 0:
        return

    if best_offset_right_deg <= 180:
        rotate_deg(direction=2, degrees=best_offset_right_deg)
    else:
        rotate_deg(direction=1, degrees=(360 - best_offset_right_deg))


def stop_handler(signum, frame):
    """Set run flag false so main loop exits and cleanup runs."""
    global _running
    _running = False


def print_controls():
    """Show keyboard controls once at startup."""
    print("")
    print("Keyboard controls:")
    print("  o : toggle manual override mode ON/OFF")
    print("  w : manual forward   (override mode)")
    print("  s : manual backward  (override mode)")
    print("  a : rotate left      (override mode)")
    print("  d : rotate right     (override mode)")
    print("  x : stop movement    (override mode)")
    print("  h : request go-home  (dock return)")
    print("  Ctrl+C : exit script")
    print("")


def handle_manual_key(key):
    """
    Execute one manual movement command for override mode.
    """
    if key == "w":
        rollereye.set_translationSpeed(MANUAL_FORWARD_SPEED_MPS)
        rollereye.set_translate(degree=0)
        print("[override] forward")
    elif key == "s":
        rollereye.set_translationSpeed(MANUAL_FORWARD_SPEED_MPS)
        rollereye.set_translate(degree=180)
        print("[override] backward")
    elif key == "a":
        rollereye.set_rotationSpeed(MANUAL_TURN_SPEED_DPS)
        rollereye.set_rotate(direction=1)
        print("[override] rotate left")
    elif key == "d":
        rollereye.set_rotationSpeed(MANUAL_TURN_SPEED_DPS)
        rollereye.set_rotate(direction=2)
        print("[override] rotate right")
    elif key == "x":
        rollereye.stop_move()
        print("[override] stop")


def log_tof_reading(range_m, timestamp_s, override_mode):
    """
    Print a compact TOF telemetry line for debugging.

    Output fields:
      mode   : AUTO or OVERRIDE
      tof_m  : latest TOF distance in meters
      age_s  : age of that reading in seconds
      state  : FRESH / STALE / NONE
    """
    now = time.time()
    mode_label = "OVERRIDE" if override_mode else "AUTO"

    if range_m is None:
        print("[TOF][%s] tof_m=NONE age_s=NONE state=NONE" % mode_label)
        return

    age_s = now - timestamp_s
    freshness = "FRESH" if age_s <= TOF_STALE_TIMEOUT_S else "STALE"
    print(
        "[TOF][%s] tof_m=%.3f age_s=%.3f state=%s"
        % (mode_label, range_m, age_s, freshness)
    )


def request_go_home():
    """
    Ask NavPath to return to home/dock.

    NavPath implements this behavior in `nav_patrol`:
      isFromOutStart = 0
      name = ""       (empty path name means "back to home")
    """
    if nav_patrol is None:
        print("[home] nav_patrol service type is unavailable in this runtime.")
        return False

    try:
        patrol_proxy = rospy.ServiceProxy(NAV_PATROL_SERVICE, nav_patrol)
        response = patrol_proxy(0, "")
    except Exception as exc:
        print("[home] failed to call %s: %s" % (NAV_PATROL_SERVICE, str(exc)))
        return False

    if response.ret == 0:
        print("[home] go-home accepted by NavPath.")
        return True
    if response.ret == 2:
        print("[home] go-home refused: low battery status reported by NavPath.")
        return False

    print("[home] go-home returned non-zero status: %s" % str(response.ret))
    return False


def request_cancel_nav():
    """
    Ask NavPath to stop its current patrol/go-home action.

    This is used when user switches from home-return mode back to manual
    override so two controllers do not compete for robot motion.
    """
    if nav_cancel is None:
        return

    try:
        cancel_proxy = rospy.ServiceProxy(NAV_CANCEL_SERVICE, nav_cancel)
        cancel_proxy()
        print("[home] navigation cancel requested.")
    except Exception as exc:
        print("[home] failed to call %s: %s" % (NAV_CANCEL_SERVICE, str(exc)))


def start():
    """
    Run endless maze roaming until Ctrl+C.

    The loop keeps trying to move forward; if blocked, it scans and reorients.
    """
    global _running

    rospy.Subscriber("/SensorNode/tof", Range, tof_callback)
    keyboard = KeyboardReader()
    keyboard.setup()
    print_controls()

    # Configure baseline movement speeds once.
    rollereye.set_translationSpeed(FORWARD_SPEED_MPS)
    rollereye.set_rotationSpeed(TURN_SPEED_DPS)

    # Begin with continuous forward movement.
    rollereye.set_translate(degree=0)
    moving_forward = True
    override_mode = False
    home_mode = False
    last_tof_log_s = 0.0

    try:
        while _running and not rospy.is_shutdown():
            key = keyboard.read_key()
            if key is not None:
                key = key.lower()

                # Request NavPath "go home" at any time.
                # We stop local movement first, then pause autonomous control so
                # this script does not fight nav controller outputs.
                if key == "h":
                    rollereye.stop_move()
                    moving_forward = False
                    override_mode = False
                    home_mode = request_go_home()
                    time.sleep(0.05)
                    continue

                # Toggle manual override mode on/off at runtime.
                if key == "o":
                    # If returning home was active, stop nav before entering
                    # manual mode to avoid mixed control sources.
                    if home_mode:
                        request_cancel_nav()
                        home_mode = False

                    override_mode = not override_mode
                    rollereye.stop_move()
                    moving_forward = False
                    if override_mode:
                        print("[mode] override ON (WASD active)")
                    else:
                        print("[mode] override OFF (autonomous mode)")
                    time.sleep(0.05)
                    continue

                # Manual commands are accepted only while override mode is on.
                if override_mode:
                    handle_manual_key(key)
                    time.sleep(0.05)
                    continue

            # When go-home is active, let NavPath controller own motion.
            # This loop still runs for key polling, telemetry, and shutdown.
            if home_mode:
                now = time.time()
                if now - last_tof_log_s >= TOF_LOG_INTERVAL_S:
                    range_m, ts = get_tof_snapshot()
                    log_tof_reading(range_m, ts, override_mode)
                    last_tof_log_s = now
                time.sleep(0.05)
                continue

            # In override mode, the keyboard fully controls motion.
            # We still keep looping for key polling and clean shutdown.
            if override_mode:
                # Keep TOF logs flowing even in override mode for visibility.
                now = time.time()
                if now - last_tof_log_s >= TOF_LOG_INTERVAL_S:
                    range_m, ts = get_tof_snapshot()
                    log_tof_reading(range_m, ts, override_mode)
                    last_tof_log_s = now
                time.sleep(0.05)
                continue

            range_m, ts = get_tof_snapshot()
            now = time.time()

            # Periodic TOF telemetry logging in autonomous mode.
            if now - last_tof_log_s >= TOF_LOG_INTERVAL_S:
                log_tof_reading(range_m, ts, override_mode)
                last_tof_log_s = now

            # If TOF is stale/missing, treat it as "no obstacle seen" and keep
            # moving forward (requested behavior for max-range/no-return cases).
            if range_m is None or (now - ts) > TOF_STALE_TIMEOUT_S:
                if not moving_forward:
                    rollereye.set_translate(degree=0)
                    moving_forward = True
                time.sleep(0.05)
                continue

            # If blocked, perform scan-based heading selection.
            if range_m <= STOP_DISTANCE_M:
                if moving_forward:
                    rollereye.stop_move()
                    moving_forward = False
                time.sleep(SETTLE_S)

                best_offset, best_dist = choose_best_heading_with_scan()
                print(
                    "Blocked at %.3fm, best heading right=%d deg (%.3fm)"
                    % (range_m, best_offset, best_dist)
                )
                turn_to_best_heading(best_offset)

                # Resume continuous forward movement from the new heading.
                rollereye.set_translate(degree=0)
                moving_forward = True

            # If currently stopped and clearance improves, resume.
            elif (not moving_forward) and range_m >= RESUME_DISTANCE_M:
                rollereye.set_translate(degree=0)
                moving_forward = True

            time.sleep(0.05)
    finally:
        # Always restore the user's terminal input mode.
        keyboard.restore()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGHUP, stop_handler)

    rollereye.start()
    # `rollereye.start()` redirects stdout to /dev/null in this SDK.
    # Re-enable stdout so debugging prints (including TOF logs) are visible.
    enable_print()
    try:
        start()
    except Exception as e:
        # Send exception details to Scout's app/debug handling path.
        rollereye.handle_exception(e.__class__.__name__ + ": " + str(e))
        raise
    finally:
        # Always stop motion and release SDK resources on exit.
        rollereye.stop()
