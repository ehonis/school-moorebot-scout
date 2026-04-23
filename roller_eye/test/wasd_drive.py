#!/usr/bin/env python3

"""
Simple keyboard teleop for Scout using WASD.

Controls:
  w : drive forward
  s : drive backward
  a : rotate left
  d : rotate right
  x : stop movement
  q : quit script
  Ctrl+C : quit script

This script is intentionally minimal:
- No obstacle avoidance
- No autonomy
- Just direct movement commands from keyboard input
"""

import select
import signal
import sys
import termios
import time
import tty


# Scout SDK is usually installed here on-device.
sys.path.append("/usr/local/lib")
from rollereye import *  # noqa: F401,F403 - SDK examples use star import


# Keep speeds conservative for safe indoor testing.
MOVE_SPEED_MPS = 0.16
TURN_SPEED_DPS = 75


# Global flag used by signal handler to end loop cleanly.
_running = True


class KeyboardReader(object):
    """
    Read one key at a time without pressing Enter.

    We put stdin into cbreak mode so key presses are available immediately.
    On shutdown we always restore original terminal settings.
    """

    def __init__(self):
        self._enabled = False
        self._fd = None
        self._old_settings = None

    def setup(self):
        """
        Enable cbreak mode if stdin is a real terminal.
        """
        if not sys.stdin.isatty():
            print("stdin is not a TTY; keyboard control unavailable.")
            return

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._enabled = True

    def read_key(self):
        """
        Return a single pressed key, or None if no key is waiting.
        """
        if not self._enabled:
            return None

        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            return sys.stdin.read(1)
        return None

    def restore(self):
        """
        Put terminal settings back to normal.
        """
        if self._enabled and self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        self._enabled = False


def stop_handler(signum, frame):
    """
    Signal handler for Ctrl+C / termination signals.
    """
    del signum, frame  # Unused, but kept for signal API compatibility.
    global _running
    _running = False


def print_controls():
    """
    Print control help once at startup.
    """
    print("")
    print("WASD controls:")
    print("  w : forward")
    print("  s : backward")
    print("  a : rotate left")
    print("  d : rotate right")
    print("  x : stop")
    print("  q : quit")
    print("  Ctrl+C : quit")
    print("")


def handle_key(key):
    """
    Execute one movement action for the given key.

    Returns:
      True  -> keep running
      False -> exit loop
    """
    if key == "w":
        # degree=0 means move along robot's forward axis.
        rollereye.set_translationSpeed(MOVE_SPEED_MPS)
        rollereye.set_translate(degree=0)
        print("forward")
        return True

    if key == "s":
        # degree=180 moves in the opposite direction (backward).
        rollereye.set_translationSpeed(MOVE_SPEED_MPS)
        rollereye.set_translate(degree=180)
        print("backward")
        return True

    if key == "a":
        # direction=1 rotates counter-clockwise (left).
        rollereye.set_rotationSpeed(TURN_SPEED_DPS)
        rollereye.set_rotate(direction=1)
        print("left")
        return True

    if key == "d":
        # direction=2 rotates clockwise (right).
        rollereye.set_rotationSpeed(TURN_SPEED_DPS)
        rollereye.set_rotate(direction=2)
        print("right")
        return True

    if key == "x":
        rollereye.stop_move()
        print("stop")
        return True

    if key == "q":
        return False

    # Ignore other keys quietly to keep teleop simple.
    return True


def run():
    """
    Main control loop: poll keyboard and send matching movement commands.
    """
    global _running

    keyboard = KeyboardReader()
    keyboard.setup()
    print_controls()

    try:
        while _running:
            key = keyboard.read_key()
            if key is not None:
                key = key.lower()
                should_continue = handle_key(key)
                if not should_continue:
                    _running = False
                    break

            # Small sleep reduces CPU usage while keeping controls responsive.
            time.sleep(0.02)
    finally:
        keyboard.restore()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGHUP, stop_handler)

    rollereye.start()
    # SDK start redirects stdout on some builds; restore normal printing.
    enable_print()

    try:
        run()
    except Exception as e:
        # Route exceptions through SDK helper for consistency with other tests.
        rollereye.handle_exception(e.__class__.__name__ + ": " + str(e))
        raise
    finally:
        # Always stop the robot and release SDK resources on exit.
        rollereye.stop_move()
        rollereye.stop()
