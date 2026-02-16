#!/usr/bin/env python3
"""
Read and optionally set STS3215 (SO-100) EEPROM angle limits.

By default servos may have restrictive Min_Angle_Limit / Max_Angle_Limit (e.g. 0–2338),
which can cause joints to stop mid-range. This script sets them to full range (0–4095)
so software/calibration controls the limits.

Usage (run from phosphobot directory):
  uv run python scripts/set_eeprom_limits.py --port /dev/ttyACM0           # read only
  uv run python scripts/set_eeprom_limits.py --port /dev/ttyACM0 --write   # read then write full range
  uv run python scripts/set_eeprom_limits.py --port /dev/ttyACM1 --write    # follower arm

Requires only one arm connected on the given port (servos 1–6).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow importing phosphobot when run as script (phosphobot/phosphobot/scripts/ -> phosphobot/phosphobot is package root)
if __name__ == "__main__" and "__file__" in dir():
    _pkg_root = Path(__file__).resolve().parent.parent
    if str(_pkg_root) not in sys.path:
        sys.path.insert(0, str(_pkg_root))

from phosphobot.hardware.motors.feetech import (
    FeetechMotorsBus,
    SCS_SERIES_CONTROL_TABLE,
)

SO100_MOTORS = {
    "shoulder_pan": (1, "sts3215"),
    "shoulder_lift": (2, "sts3215"),
    "elbow_flex": (3, "sts3215"),
    "wrist_flex": (4, "sts3215"),
    "wrist_roll": (5, "sts3215"),
    "gripper": (6, "sts3215"),
}

MIN_ADDR, MIN_BYTES = SCS_SERIES_CONTROL_TABLE["Min_Angle_Limit"]
MAX_ADDR, MAX_BYTES = SCS_SERIES_CONTROL_TABLE["Max_Angle_Limit"]
FULL_MIN, FULL_MAX = 0, 4095


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read and optionally set SO-100/STS3215 EEPROM angle limits to full range (0–4095)."
    )
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port (e.g. /dev/ttyACM0 or /dev/ttyACM1)",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write Min=0 and Max=4095 to all servos (unlocks EEPROM, then writes).",
    )
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=SO100_MOTORS)
    try:
        bus.connect()
    except Exception as e:
        print(f"Failed to connect to {args.port}: {e}")
        print("Ensure only one SO-100 arm is connected on this port and no other process is using it.")
        sys.exit(1)

    try:
        # Unlock EEPROM so we can write to Min/Max angle limits
        bus.write("Lock", 0)
        print("Unlocked EEPROM (Lock=0).")

        min_limits = bus.read("Min_Angle_Limit")
        max_limits = bus.read("Max_Angle_Limit")

        print("\nCurrent EEPROM angle limits (servo 1–6):")
        print("  Servo     Min_Angle_Limit  Max_Angle_Limit")
        for i, name in enumerate(SO100_MOTORS):
            print(f"  {i+1} {name:14} {min_limits[i]:>10}        {max_limits[i]:>10}")

        if not args.write:
            print("\nNo write requested. Use --write to set all to Min=0, Max=4095.")
            return

        # Set full range for all 6 servos
        bus.write("Min_Angle_Limit", [FULL_MIN] * 6)
        bus.write("Max_Angle_Limit", [FULL_MAX] * 6)
        print(f"\nWritten Min_Angle_Limit={FULL_MIN}, Max_Angle_Limit={FULL_MAX} for all servos.")

        # Verify
        min_after = bus.read("Min_Angle_Limit")
        max_after = bus.read("Max_Angle_Limit")
        ok = all(m == FULL_MIN for m in min_after) and all(m == FULL_MAX for m in max_after)
        if ok:
            print("Verified: limits updated successfully.")
        else:
            print("Warning: read-back differs from expected:", min_after, max_after)
    finally:
        if bus.is_connected:
            bus.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
