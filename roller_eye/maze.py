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
STOP_DISTANCE_M = 0.2

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


class MovementRecorder(object):
    """
    Keep a time-based history of motion commands so we can backtrack.

    Why time-based?
      - Many movement calls in this script are continuous (translate/rotate)
        until another command interrupts them.
      - To reverse that motion, we need both direction and how long it ran.
    """

    def __init__(self):
        # Finished movement segments/events in execution order.
        self._history = []

        # Currently active continuous motion segment (or None).
        self._active = None

        # Guard to avoid recording reverse-playback commands.
        self._is_replaying = False

    def _finish_active(self):
        """
        Close out the active continuous segment and store duration.
        """
        if self._active is None:
            return

        duration = time.time() - self._active["start_s"]
        if duration > 0.0:
            segment = dict(self._active)
            segment["duration_s"] = duration
            self._history.append(segment)
        self._active = None

    def start_translate(self, degree, speed_mps):
        """
        Start continuous translation and begin timing this segment.
        """
        rollereye.set_translationSpeed(speed_mps)
        rollereye.set_translate(degree=degree)
        if self._is_replaying:
            return
        self._finish_active()
        self._active = {
            "kind": "translate",
            "degree": degree,
            "speed_mps": speed_mps,
            "start_s": time.time(),
        }

    def start_rotate(self, direction, speed_dps):
        """
        Start continuous rotation and begin timing this segment.
        """
        rollereye.set_rotationSpeed(speed_dps)
        rollereye.set_rotate(direction=direction)
        if self._is_replaying:
            return
        self._finish_active()
        self._active = {
            "kind": "rotate",
            "direction": direction,
            "speed_dps": speed_dps,
            "start_s": time.time(),
        }

    def rotate_degrees(self, direction, degrees, speed_dps, record=True):
        """
        Execute an exact-degree turn and record it as a discrete event.
        """
        rollereye.set_rotationSpeed(speed_dps)
        rollereye.set_rotate_3(direction=direction, degree=degrees)
        time.sleep(SETTLE_S)
        if self._is_replaying or (not record):
            return
        self._finish_active()
        self._history.append(
            {
                "kind": "rotate_deg",
                "direction": direction,
                "degrees": degrees,
                "speed_dps": speed_dps,
            }
        )

    def stop_motion(self):
        """
        Stop the robot and finalize any running continuous segment.
        """
        rollereye.stop_move()
        if self._is_replaying:
            return
        self._finish_active()

    def replay_reverse(self):
        """
        Turn around, then replay history in reverse.

        Turning around first lets reverse traversal use forward translation,
        so the robot does not spend the whole replay driving backward.
        """
        self._finish_active()
        if not self._history:
            print("[history] no movement recorded yet.")
            return

        print("[history] reversing %d recorded steps..." % len(self._history))
        self._is_replaying = True
        try:
            # Face back along the route before replaying reverse actions.
            rollereye.set_rotationSpeed(TURN_SPEED_DPS)
            rollereye.set_rotate_3(direction=2, degree=180)
            time.sleep(SETTLE_S)

            for step in reversed(self._history):
                if step["kind"] == "translate":
                    # Because we already turned 180, replaying translate with
                    # the same local direction retraces the path forward.
                    rollereye.set_translationSpeed(step["speed_mps"])
                    rollereye.set_translate(degree=step["degree"])
                    time.sleep(step["duration_s"])
                    rollereye.stop_move()
                    time.sleep(SETTLE_S)
                elif step["kind"] == "rotate":
                    inverse_direction = 2 if step["direction"] == 1 else 1
                    rollereye.set_rotationSpeed(step["speed_dps"])
                    rollereye.set_rotate(direction=inverse_direction)
                    time.sleep(step["duration_s"])
                    rollereye.stop_move()
                    time.sleep(SETTLE_S)
                elif step["kind"] == "rotate_deg":
                    inverse_direction = 2 if step["direction"] == 1 else 1
                    rollereye.set_rotationSpeed(step["speed_dps"])
                    rollereye.set_rotate_3(
                        direction=inverse_direction, degree=step["degrees"]
                    )
                    time.sleep(SETTLE_S)
        finally:
            self._is_replaying = False

        print("[history] reverse replay complete.")


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


def wait_for_fresh_tof(max_wait_s, keyboard=None, recorder=None, override_mode=False):
    """
    Wait briefly for a fresh TOF value and return it.

    Returns:
      (distance_m, override_mode, interrupted)
      - distance_m: float meters or None if no fresh value arrived.
      - override_mode: potentially-updated mode after key processing.
      - interrupted: True when a key command should abort current autonomy step.
    """
    start = time.time()
    while time.time() - start <= max_wait_s:
        # During scan/turn workflows we can still poll keyboard here so the
        # user does not need to wait for this wait loop to finish.
        if keyboard is not None and recorder is not None:
            key = keyboard.read_key()
            if key is not None:
                override_mode, _, should_interrupt = process_key_command(
                    key=key, recorder=recorder, override_mode=override_mode
                )
                if should_interrupt:
                    return None, override_mode, True

        range_m, ts = get_tof_snapshot()
        if range_m is not None and (time.time() - ts) <= TOF_STALE_TIMEOUT_S:
            return range_m, override_mode, False
        time.sleep(0.01)
    return None, override_mode, False


def rotate_deg(recorder, direction, degrees, record=True):
    """
    Rotate by exact degrees using Scout API.

    direction:
      1 = left (counter-clockwise)
      2 = right (clockwise)
    """
    recorder.rotate_degrees(
        direction=direction,
        degrees=degrees,
        speed_dps=TURN_SPEED_DPS,
        record=record,
    )


def choose_best_heading_with_scan(recorder, keyboard, override_mode):
    """
    Scan full 360 degrees and choose the best heading.

    We sample center (0 deg), then rotate right one step at a time through
    all remaining headings, and finish by returning to center heading.

    Returns:
      best_offset_right_deg: heading offset (from original), turning right.
      best_distance_m: measured clearance at that heading.
      override_mode: possibly-updated mode if user toggled during scan.
      interrupted: True when a user key command should abort autonomous scan.
    """
    steps = int(360 / SCAN_STEP_DEG)

    # Measure current heading first (offset = 0).
    best_distance, override_mode, interrupted = wait_for_fresh_tof(
        TOF_WAIT_TIMEOUT_S,
        keyboard=keyboard,
        recorder=recorder,
        override_mode=override_mode,
    )
    if interrupted:
        return 0, 0.0, override_mode, True
    if best_distance is None:
        best_distance = 0.0
    best_offset_right_deg = 0

    # Step through all other headings to the right.
    for step_idx in range(1, steps):
        key = keyboard.read_key()
        if key is not None:
            override_mode, _, should_interrupt = process_key_command(
                key=key, recorder=recorder, override_mode=override_mode
            )
            if should_interrupt:
                return best_offset_right_deg, best_distance, override_mode, True

        # Scan spins are sensing-only actions and should not be added to
        # backtrack history; h should replay travel, not replay a scan.
        rotate_deg(
            recorder=recorder, direction=2, degrees=SCAN_STEP_DEG, record=False
        )
        d, override_mode, interrupted = wait_for_fresh_tof(
            TOF_WAIT_TIMEOUT_S,
            keyboard=keyboard,
            recorder=recorder,
            override_mode=override_mode,
        )
        if interrupted:
            return best_offset_right_deg, best_distance, override_mode, True
        if d is None:
            d = 0.0

        offset = step_idx * SCAN_STEP_DEG
        if d > best_distance:
            best_distance = d
            best_offset_right_deg = offset

    # Final step returns robot to center heading.
    rotate_deg(recorder=recorder, direction=2, degrees=SCAN_STEP_DEG, record=False)

    return best_offset_right_deg, best_distance, override_mode, False


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


def turn_to_best_heading(recorder, keyboard, override_mode, best_offset_right_deg):
    """
    Turn from current heading to selected heading with shortest direction.

    Returns:
      (override_mode, interrupted)
      - interrupted is True if a keyboard command interrupted this turn.
    """
    if best_offset_right_deg == 0:
        return override_mode, False

    if best_offset_right_deg <= 180:
        direction = 2
        remaining_deg = best_offset_right_deg
    else:
        direction = 1
        remaining_deg = 360 - best_offset_right_deg

    # Turn in small chunks so we can poll keyboard between chunks.
    while remaining_deg > 0:
        key = keyboard.read_key()
        if key is not None:
            override_mode, _, should_interrupt = process_key_command(
                key=key, recorder=recorder, override_mode=override_mode
            )
            if should_interrupt:
                return override_mode, True

        step_deg = min(SCAN_STEP_DEG, remaining_deg)
        rotate_deg(recorder=recorder, direction=direction, degrees=step_deg)
        remaining_deg -= step_deg

    return override_mode, False


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
    print("  h : reverse recorded movement path")
    print("  Ctrl+C : exit script")
    print("")


def handle_manual_key(key, recorder):
    """
    Execute one manual movement command for override mode.
    """
    if key == "w":
        recorder.start_translate(degree=0, speed_mps=MANUAL_FORWARD_SPEED_MPS)
        print("[override] forward")
    elif key == "s":
        recorder.start_translate(degree=180, speed_mps=MANUAL_FORWARD_SPEED_MPS)
        print("[override] backward")
    elif key == "a":
        recorder.start_rotate(direction=1, speed_dps=MANUAL_TURN_SPEED_DPS)
        print("[override] rotate left")
    elif key == "d":
        recorder.start_rotate(direction=2, speed_dps=MANUAL_TURN_SPEED_DPS)
        print("[override] rotate right")
    elif key == "x":
        recorder.stop_motion()
        print("[override] stop")


def process_key_command(key, recorder, override_mode):
    """
    Process one keyboard command and report whether autonomy should pause.

    Returns:
      override_mode: possibly-updated override flag.
      was_handled: True when key matched a known command.
      should_interrupt: True when caller should stop autonomous scan/turn flow.
    """
    key = key.lower()

    # h should always work, even when autonomous scan is mid-flight.
    if key == "h":
        recorder.stop_motion()
        recorder.replay_reverse()
        return override_mode, True, True

    # o toggles manual mode and should immediately halt autonomous actions.
    if key == "o":
        override_mode = not override_mode
        recorder.stop_motion()
        if override_mode:
            print("[mode] override ON (WASD active)")
        else:
            print("[mode] override OFF (autonomous mode)")
        return override_mode, True, True

    # In override mode, movement keys should execute immediately.
    if override_mode:
        handle_manual_key(key, recorder)
        return override_mode, True, True

    return override_mode, False, False


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
    recorder = MovementRecorder()

    # Begin with continuous forward movement.
    recorder.start_translate(degree=0, speed_mps=FORWARD_SPEED_MPS)
    moving_forward = True
    override_mode = False

    try:
        while _running and not rospy.is_shutdown():
            key = keyboard.read_key()
            if key is not None:
                override_mode, _, should_interrupt = process_key_command(
                    key=key, recorder=recorder, override_mode=override_mode
                )
                if should_interrupt:
                    # Any immediate command means autonomous movement decisions
                    # should pause until the next loop iteration.
                    moving_forward = False
                    time.sleep(0.05)
                    continue

            # In override mode, the keyboard fully controls motion.
            # We still keep looping for key polling and clean shutdown.
            if override_mode:
                time.sleep(0.05)
                continue

            range_m, ts = get_tof_snapshot()
            now = time.time()

            # If TOF is stale/missing, treat it as "no obstacle seen" and keep
            # moving forward (requested behavior for max-range/no-return cases).
            if range_m is None or (now - ts) > TOF_STALE_TIMEOUT_S:
                if not moving_forward:
                    recorder.start_translate(degree=0, speed_mps=FORWARD_SPEED_MPS)
                    moving_forward = True
                time.sleep(0.05)
                continue

            # If blocked, perform scan-based heading selection.
            if range_m <= STOP_DISTANCE_M:
                if moving_forward:
                    recorder.stop_motion()
                    moving_forward = False
                time.sleep(SETTLE_S)

                best_offset, _, override_mode, interrupted = (
                    choose_best_heading_with_scan(
                        recorder=recorder,
                        keyboard=keyboard,
                        override_mode=override_mode,
                    )
                )
                if interrupted:
                    moving_forward = False
                    time.sleep(0.05)
                    continue
                override_mode, interrupted = turn_to_best_heading(
                    recorder=recorder,
                    keyboard=keyboard,
                    override_mode=override_mode,
                    best_offset_right_deg=best_offset,
                )
                if interrupted:
                    moving_forward = False
                    time.sleep(0.05)
                    continue

                # Resume continuous forward movement from the new heading.
                recorder.start_translate(degree=0, speed_mps=FORWARD_SPEED_MPS)
                moving_forward = True

            # If currently stopped and clearance improves, resume.
            elif (not moving_forward) and range_m >= RESUME_DISTANCE_M:
                recorder.start_translate(degree=0, speed_mps=FORWARD_SPEED_MPS)
                moving_forward = True

            time.sleep(0.05)
    finally:
        recorder.stop_motion()
        # Always restore the user's terminal input mode.
        keyboard.restore()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGHUP, stop_handler)

    rollereye.start()
    # `rollereye.start()` redirects stdout to /dev/null in this SDK.
    # Re-enable stdout so control/status prints are visible.
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
