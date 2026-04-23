# -*- coding: utf-8 -*-
"""
Simple Scout movement script:
1) Move forward about one foot.
2) Turn around by 180 degrees.

This follows the usage style shown in the project README and uses the
high-level movement helpers provided by the `rollereye` module.
"""

import sys
import time


# On the Scout device, `rollereye.py` is usually installed under /usr/local/lib.
# This keeps the script usable in the expected runtime environment.
sys.path.append("/usr/local/lib")

from rollereye import *  # noqa: F401,F403 - project examples use star import


# Start the robot SDK/runtime before issuing movement commands.
rollereye.start()


def start():
    """
    Execute a short movement sequence:
    - Drive forward 1 foot (0.3048 meters)
    - Rotate left 180 degrees
    """
    # 1 foot in meters. This is the target forward distance.
    one_foot_meters = 0.3048

    # Translation direction:
    # - In this SDK, 0 degrees means "forward".
    # Translation speed is in m/s (allowed range is 0~1).
    rollereye.set_translationSpeed(0.2)
    rollereye.set_translate_3(degree=0, meters=one_foot_meters)

    # Small settle delay between motion segments so the action is visibly
    # separated and we avoid command overlap on slower systems.
    time.sleep(0.25)

    # Rotation setup:
    # - Rotation speed is in degrees/s.
    # - Direction 1 = left (counter-clockwise), 2 = right (clockwise).
    # - 180 degrees is a full "turn around".
    rollereye.set_rotationSpeed(90)
    rollereye.set_rotate_3(direction=1, degree=180)

    # Ensure there is no lingering async motion command.
    rollereye.stop_move()


if __name__ == "__main__":
    try:
        start()
    except Exception as e:
        # Forward runtime errors to Scout's programming exception handler so
        # errors are visible in the robot app/debug pipeline.
        rollereye.handle_exception(e.__class__.__name__ + ": " + str(e))
    finally:
        # Always release robot resources and exit cleanly.
        rollereye.stop()
