import re
import signal
import sys
import time


# The Scout Python API is typically available here on-device.
sys.path.append("/usr/local/lib")
from rollereye import *  # noqa: F401,F403 - project examples use star import


# Keep these speeds conservative so motion is easier to observe and tune.
FORWARD_SPEED_MPS = 0.20
TURN_SPEED_DPS = 90

# Short pause between commands so command boundaries are clean.
STEP_GAP_SECONDS = 0.20


def print_usage_and_exit(code=1):
    """Show CLI usage and exit."""
    print("Usage:")
    print("  python run_motion_sequence.py 1f 90r 0.5f 45l")
    print("  python run_motion_sequence.py \"1f,90r,0.5f,45l\"")
    print("")
    print("Rules:")
    print("  - Number can be decimal (e.g. 0.25f)")
    print("  - f = meters forward")
    print("  - r/l = degrees right/left")
    print("  - Commands run in the exact order given")
    sys.exit(code)


def normalize_tokens(argv_items):
    """
    Convert raw CLI arguments into a clean list of command tokens.

    Supports both:
    - Space separated: 1f 90r 0.5f
    - Comma separated: "1f,90r,0.5f"
    """
    raw = " ".join(argv_items).replace(",", " ")
    tokens = [t.strip() for t in raw.split() if t.strip()]
    return tokens


def parse_token(token):
    """
    Parse one command token into (value, action).

    Accepted token examples: 1f, 0.5f, 90r, 45l
    """
    m = re.match(r"^([0-9]+(?:\.[0-9]+)?)([fFrRlL])$", token)
    if not m:
        raise ValueError("Invalid token: %s" % token)

    value = float(m.group(1))
    action = m.group(2).lower()

    # Basic safety validation to catch accidental zeros/negatives.
    if value <= 0:
        raise ValueError("Value must be > 0 in token: %s" % token)

    return value, action


def execute_sequence(tokens):
    """Execute parsed commands in sequence using rollereye motion APIs."""
    # Configure motion speeds once up-front.
    rollereye.set_translationSpeed(FORWARD_SPEED_MPS)
    rollereye.set_rotationSpeed(TURN_SPEED_DPS)

    for token in tokens:
        value, action = parse_token(token)

        # Command tracing helps while testing from SSH/terminal.
        print("Executing:", token)

        if action == "f":
            # Move forward by `value` meters.
            # degree=0 is forward in this SDK.
            rollereye.set_translate_3(degree=0, meters=value)
        elif action == "r":
            # Turn right by `value` degrees.
            # direction=2 means right/clockwise.
            rollereye.set_rotate_3(direction=2, degree=value)
        elif action == "l":
            # Turn left by `value` degrees.
            # direction=1 means left/counter-clockwise.
            rollereye.set_rotate_3(direction=1, degree=value)
        else:
            # Should never happen because parse_token validates action.
            raise ValueError("Unsupported action: %s" % action)

        # Small settle time keeps movement transitions clean.
        time.sleep(STEP_GAP_SECONDS)

    # Ensure there is no lingering async motion command.
    rollereye.stop_move()


def _signal_handler(signum, frame):
    """Stop the robot quickly if user interrupts the script."""
    try:
        rollereye.stop_move()
    finally:
        rollereye.stop()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)

    if len(sys.argv) < 2:
        print_usage_and_exit(1)

    tokens = normalize_tokens(sys.argv[1:])
    if not tokens:
        print_usage_and_exit(1)

    rollereye.start()
    try:
        execute_sequence(tokens)
        print("Sequence finished. Keeping rollereye open. Press Ctrl+C to exit.")
        while True:
            # Keep process alive so the runtime stays open until user stops it.
            time.sleep(1.0)
    except Exception as e:
        # Surface exceptions to Scout's app/debug pipeline.
        rollereye.handle_exception(e.__class__.__name__ + ": " + str(e))
        raise
