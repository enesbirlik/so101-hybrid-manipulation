#!/usr/bin/env python3
"""Keyboard joint teleop for the SO-101 (no ROS, no MoveIt).

  q / a  shoulder_pan     w / s  shoulder_lift    e / d  elbow_flex
  r / f  wrist_flex       t / g  wrist_roll       y / h  gripper
  z      go to zero (lift, then elbow, then the rest)
  space  stop where it is            + / -  bigger / smaller step
  x      quit (asks before torque off)   Ctrl+C  torque off at once

Starts by holding the arm where it is. Goals stay inside the calibrated range.
A watchdog freezes the arm and prints the servo state when a joint falls more
than SAG_DEG behind its goal (the arm sagging or blocked).

Opens the serial bus directly: never run it while so101_hardware is running.
"""

import argparse
import os
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from pathlib import Path

import feetech_bus as fb
from feetech_bus import FeetechBus, FeetechError
from so101_calibrate import (DEFAULT_FILE, JOINTS, LIMIT_MARGIN_DEG, TICKS_PER_DEG, Abort, go_to_zero,
                             hold_all, load_calibration, require_servos, to_deg)

KEYS = {"q": (0, +1), "a": (0, -1), "w": (1, +1), "s": (1, -1), "e": (2, +1), "d": (2, -1),
        "r": (3, +1), "f": (3, -1), "t": (4, +1), "g": (4, -1), "y": (5, +1), "h": (5, -1)}
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
         "wrist_flex": "wflex", "wrist_roll": "roll", "gripper": "grip"}
STEPS_DEG = [0.5, 1.0, 2.0, 5.0, 10.0]
LEAD_DEG = 8.0     # a goal never runs further ahead of the joint than this
SAG_DEG = 15.0     # watchdog: this far from the goal ...
SAG_TIME = 0.5     # ... for this long [s] means the joint is sagging or blocked
LOOP_HZ = 30


@contextmanager
def cbreak():
    """Single key presses without Enter; Ctrl+C still raises KeyboardInterrupt."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_keys(timeout):
    fd = sys.stdin.fileno()
    if select.select([fd], [], [], timeout)[0]:
        return os.read(fd, 64).decode(errors="ignore")
    return ""


def say(msg):
    sys.stdout.write(f"\r\x1b[2K{msg}\n")


class Teleop:
    def __init__(self, bus, cal):
        self.bus = bus
        self.cal = cal
        self.joints = [j for j in JOINTS if j.name in cal]
        margin = round(LIMIT_MARGIN_DEG * TICKS_PER_DEG)
        self.limits = {j.name: (cal[j.name]["range_min"] + margin, cal[j.name]["range_max"] - margin)
                       for j in self.joints}
        self.step = 2
        self.goal = {}
        self.sag_since = {}

    def positions(self):
        return {j.name: self.bus.read_position(j.id) for j in self.joints}

    def set_goal(self, j, ticks):
        lo, hi = self.limits[j.name]
        self.goal[j.name] = min(max(ticks, lo), hi)
        self.bus.write_u16(j.id, fb.GOAL_POSITION, self.goal[j.name])

    def freeze(self):
        for j, ticks in zip(self.joints, self.positions().values()):
            self.set_goal(j, ticks)
        self.sag_since.clear()

    def jog(self, j, sign):
        pos = self.bus.read_position(j.id)
        lead = round(LEAD_DEG * TICKS_PER_DEG)
        target = self.goal[j.name] + round(sign * STEPS_DEG[self.step] * TICKS_PER_DEG)
        self.set_goal(j, min(max(target, pos - lead), pos + lead))

    def watchdog(self, pos):
        now = time.monotonic()
        for j in self.joints:
            if abs(pos[j.name] - self.goal[j.name]) <= SAG_DEG * TICKS_PER_DEG:
                self.sag_since.pop(j.name, None)
                continue
            if now - self.sag_since.setdefault(j.name, now) < SAG_TIME:
                continue
            r = self.bus.read(j.id, fb.TORQUE_ENABLE, 31)  # addresses 40..70
            load = fb.decode_sm(r[20] | r[21] << 8, 10) / 10
            current = (r[29] | r[30] << 8) * 6.5
            say(f"! {j.name}: {to_deg(pos[j.name], self.cal[j.name]):+.1f}° but goal "
                f"{to_deg(self.goal[j.name], self.cal[j.name]):+.1f}°  torque_enable {r[0]}  load {load:+.1f} %  "
                f"current {current:.0f} mA  {r[22] / 10:.1f} V  {r[23]} C  status {r[25]:08b}")
            say("  frozen in place. Tell Claude these numbers.")
            self.freeze()
            return

    def status(self, pos):
        joints = "  ".join(f"{SHORT[j.name]} {to_deg(pos[j.name], self.cal[j.name]):+6.1f}" for j in self.joints)
        sys.stdout.write(f"\r\x1b[2Kstep {STEPS_DEG[self.step]:>4}°  {joints}")
        sys.stdout.flush()

    def run(self):
        hold_all(self.bus, self.joints)
        self.freeze()
        say("holding where it is. z = go to zero, x = quit.")
        quitting = False
        while True:
            for ch in read_keys(1 / LOOP_HZ):
                if quitting:
                    if ch == "x":
                        return
                    quitting = False
                    say("quit cancelled.")
                elif ch in KEYS:
                    idx, sign = KEYS[ch]
                    if JOINTS[idx] in self.joints:
                        self.jog(JOINTS[idx], sign)
                elif ch == "z":
                    say("going to zero:")
                    stuck = go_to_zero(self.bus, self.cal, self.joints)
                    say(f"stuck: {', '.join(stuck)}. Frozen in place." if stuck else "at zero.")
                    self.freeze()
                elif ch == " ":
                    self.freeze()
                elif ch in "+=":
                    self.step = min(self.step + 1, len(STEPS_DEG) - 1)
                elif ch == "-":
                    self.step = max(self.step - 1, 0)
                elif ch == "x":
                    quitting = True
                    say("Support the arm. x again = torque off and quit, any other key = keep going.")
            try:
                pos = self.positions()
            except FeetechError:
                continue  # a missed reply; try again next cycle
            self.watchdog(pos)
            self.status(pos)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"calibration file (default: {DEFAULT_FILE})")
    args = parser.parse_args()

    cal = load_calibration(args.file)
    if not cal:
        print(f"error: no calibration in {args.file}", file=sys.stderr)
        return 1
    try:
        with FeetechBus(args.port or fb.find_port()) as bus:
            require_servos(bus, [j for j in JOINTS if j.name in cal])
            print(__doc__.split("\n\n")[1])
            try:
                with cbreak():
                    Teleop(bus, cal).run()
            except KeyboardInterrupt:
                pass
            finally:
                for j in JOINTS:
                    bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
                print("\ntorque off.")
    except (FeetechError, Abort) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
