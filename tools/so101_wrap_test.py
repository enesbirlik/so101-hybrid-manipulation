#!/usr/bin/env python3
"""Encoder-wrap test: does a joint run away where its raw encoder wraps?

Theory (devlog day 1): the horns were not mounted at servo centre, so the raw
encoder value (present + homing_offset) wraps from 4095 to 0 inside some joint
ranges. If the servo's position loop works on the raw value, a goal on the other
side of the wrap makes it drive the long way round, into the hard stop.

The test sweeps one joint slowly, at low torque, start -> target -> start, across
its predicted wrap point. The other joints hold where they are. The tested joint
goes limp as soon as it falls behind its goal or moves against the sweep.

  python3 tools/so101_wrap_test.py                       # shoulder_pan 0 -> -20 deg at 30 %
  python3 tools/so101_wrap_test.py --joint wrist_roll --start 15 --target 35

Put the arm by hand in a raised pose where the tested joint can turn freely,
then start the script. Every sample is logged to ~/so101_data/wrap_test/.
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import feetech_bus as fb
from feetech_bus import FeetechError
from so101_calibrate import (DEFAULT_FILE, JOINTS, MOVE_ACC, TICKS_PER_DEG, Abort, LiveView,
                             load_calibration, open_bus, require_servos, to_deg)

LOG_DIR = Path.home() / "so101_data" / "wrap_test"
LOOP_HZ = 20
SWEEP_DEG_S = 5.0     # goal ramp speed
LAG_DEG = 4.0         # pos this far behind the goal: stop
BACKWARD_DEG = 2.0    # pos this far against the sweep direction: stop
HOLD_MOVE_DEG = 3.0   # --hold: the joint should not move at all; this much and it goes limp


def wrap_angle(bus, j, c):
    """Joint angle where the raw encoder wraps, or None if it is outside the calibrated range."""
    present = -bus.read_homing_offset(j.id) % fb.TICKS_PER_TURN
    return to_deg(present, c) if c["range_min"] <= present <= c["range_max"] else None


def read_state(bus, servo_id):
    r = bus.read(servo_id, fb.PRESENT_POSITION, 15)  # addresses 56..70
    pos = fb.decode_sm(r[0] | r[1] << 8, 15)
    load = fb.decode_sm(r[4] | r[5] << 8, 10) / 10
    status = r[9]                                     # 65
    current = (r[13] | r[14] << 8) * 6.5              # 69..70, 6.5 mA per unit
    return pos, load, status, current


def sweep(bus, j, c, offset, path, log, view):
    """Ramp the goal along `path` (degrees). Returns None, or why the joint was stopped."""
    t0 = time.monotonic()
    for a, b in zip(path, path[1:]):
        sign = 1 if b > a else -1
        n = max(1, round(abs(b - a) / SWEEP_DEG_S * LOOP_HZ))
        best = None  # furthest position along the sweep so far
        for k in range(1, n + 1):
            goal_deg = a + (b - a) * k / n
            goal = round(c["zero"] + c["direction"] * goal_deg * TICKS_PER_DEG)
            bus.write_u16(j.id, fb.GOAL_POSITION, goal)
            time.sleep(1 / LOOP_HZ)
            try:
                pos, load, status, current = read_state(bus, j.id)
            except FeetechError:
                continue
            deg = to_deg(pos, c)
            raw = (pos + offset) % fb.TICKS_PER_TURN
            log.write(f"{time.monotonic() - t0:.3f},{goal_deg:.2f},{deg:.2f},{pos},{raw},{load:.1f},"
                      f"{current:.0f},{status}\n")
            log.flush()  # the run can end in a crash or a long wait at the prompt
            view.show([f"  goal {goal_deg:+7.1f}°  pos {deg:+7.1f}°  raw {raw:4d}  load {load:+6.1f} %  "
                       f"current {current:5.0f} mA  status {status:08b}"])
            best = deg if best is None else (max if sign > 0 else min)(best, deg)
            if abs(deg - goal_deg) > LAG_DEG and k > LOOP_HZ:
                return f"lagging: goal {goal_deg:+.1f}°, at {deg:+.1f}°, raw {raw}, status {status:08b}"
            if sign * (best - deg) > BACKWARD_DEG:
                return f"moving against the sweep: goal {goal_deg:+.1f}°, at {deg:+.1f}°, raw {raw}"
        # let it settle at the end of each leg
        time.sleep(0.5)
    return None


def hold(bus, j, c, offset, log, view, seconds=3.0):
    """Torque on where the joint is now and watch it. Returns None, or why it was stopped."""
    start = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(1 / LOOP_HZ)
        try:
            pos, load, status, current = read_state(bus, j.id)
        except FeetechError:
            continue
        deg = to_deg(pos, c)
        start = deg if start is None else start
        raw = (pos + offset) % fb.TICKS_PER_TURN
        log.write(f"{time.monotonic() - t0:.3f},{start:.2f},{deg:.2f},{pos},{raw},{load:.1f},{current:.0f},{status}\n")
        log.flush()
        view.show([f"  pos {deg:+7.1f}°  moved {deg - start:+6.1f}°  load {load:+6.1f} %  "
                   f"current {current:5.0f} mA  status {status:08b}"])
        if abs(deg - start) > HOLD_MOVE_DEG:
            return f"moved {deg - start:+.1f}° ({start:+.1f}° -> {deg:+.1f}°), current {current:.0f} mA, status {status:08b}"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"calibration file (default: {DEFAULT_FILE})")
    parser.add_argument("--joint", default="shoulder_pan", choices=[j.name for j in JOINTS])
    parser.add_argument("--start", type=float, default=0.0, help="deg, safe side of the wrap")
    parser.add_argument("--target", type=float, default=-20.0, help="deg, across the wrap")
    parser.add_argument("--torque", type=int, default=30, help="%% of max torque for the tested joint")
    parser.add_argument("--hold", action="store_true",
                        help="no sweep: torque on the tested joint only, where it is now (rest pose), and watch it")
    args = parser.parse_args()

    cal = load_calibration(args.file)
    j = next(j for j in JOINTS if j.name == args.joint)
    c = cal[j.name]
    if args.hold:
        return main_hold(args, j, c)
    for a in (args.start, args.target):
        ticks = c["zero"] + c["direction"] * a * TICKS_PER_DEG
        if not c["range_min"] <= ticks <= c["range_max"]:
            raise Abort(f"{a:+.1f}° is outside the calibrated range of {j.name}")

    joints = [k for k in JOINTS if k.name in cal]
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{j.name}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open_bus(args) as bus, open(log_path, "w") as log:
        require_servos(bus, joints)
        offset = bus.read_homing_offset(j.id)
        w = wrap_angle(bus, j, c)
        now = to_deg(bus.read_position(j.id), c)
        print(f"{j.name}: now {now:+.1f}°, raw encoder wraps at "
              + (f"{w:+.1f}°" if w is not None else "a point outside its range (no runaway expected)"))
        print(f"sweep {now:+.1f}° -> {args.start:+.1f}° -> {args.target:+.1f}° -> {args.start:+.1f}° "
              f"at {args.torque} % torque; the other joints hold where they are.")
        input("Arm in a raised pose, nothing in the way? Enter to start, Ctrl+C to quit... ")
        log.write(f"# {j.name} homing_offset {offset} zero {c['zero']} wrap_deg {w}\n"
                  "t,goal_deg,pos_deg,ticks,raw,load_pct,current_mA,status\n")

        for k in joints:
            bus.write_u8(k.id, fb.ACCELERATION, MOVE_ACC)
            bus.write_u16(k.id, fb.GOAL_SPEED, 400)
            bus.write_u16(k.id, fb.TORQUE_LIMIT, args.torque * 10 if k is j else 1000)
            bus.torque(k.id, True)

        view = LiveView()
        try:
            reason = sweep(bus, j, c, offset, [now, args.start, args.target, args.start], log, view)
        except KeyboardInterrupt:
            reason = "interrupted"
        bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
        if reason:
            print(f"\nSTOPPED, {j.name} torque off: {reason}")
        else:
            print(f"\n{j.name} followed the whole sweep across {w:+.1f}°: no runaway."
                  if w is not None else f"\n{j.name} followed the whole sweep.")
        print(f"log: {log_path}")
        try:
            input("Support the arm, then press Enter to switch torque off everywhere... ")
        finally:
            for k in joints:
                bus.write_u8(k.id, fb.TORQUE_ENABLE, 0)
            print("torque off.")
    return 0


def main_hold(args, j, c):
    """Rest-pose test: is the joint outside the servo's own position limits, and does it jump?"""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{j.name}_hold_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open_bus(args) as bus, open(log_path, "w") as log:
        require_servos(bus, [j])
        for k in JOINTS:
            bus.write_u8(k.id, fb.TORQUE_ENABLE, 0)
        offset = bus.read_homing_offset(j.id)
        lo, hi = bus.read_u16(j.id, fb.MIN_POSITION_LIMIT), bus.read_u16(j.id, fb.MAX_POSITION_LIMIT)
        pos = bus.read_position(j.id)
        where = "INSIDE" if lo <= pos <= hi else "OUTSIDE"
        print(f"{j.name}: now {to_deg(pos, c):+.1f}° (tick {pos}), servo limits {lo} .. {hi} "
              f"({to_deg(lo, c):+.1f}° .. {to_deg(hi, c):+.1f}°) -> {where} the limits")
        print(f"torque on {j.name} only, at {args.torque} %, for 3 s; every other joint stays off.")
        input("Arm resting in its folded pose? Enter to start, Ctrl+C to quit... ")
        log.write(f"# {j.name} hold, homing_offset {offset} limits {lo} {hi} start_tick {pos} {where}\n"
                  "t,start_deg,pos_deg,ticks,raw,load_pct,current_mA,status\n")
        bus.write_u8(j.id, fb.ACCELERATION, MOVE_ACC)
        bus.write_u16(j.id, fb.GOAL_SPEED, 400)
        bus.write_u16(j.id, fb.TORQUE_LIMIT, args.torque * 10)
        view = LiveView()
        try:
            bus.torque(j.id, True)
            reason = hold(bus, j, c, offset, log, view)
        except KeyboardInterrupt:
            reason = "interrupted"
        finally:
            bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
        print(f"\nJUMPED, torque off: {reason}" if reason else f"\n{j.name} held still for 3 s. Torque off.")
        print(f"log: {log_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FeetechError, Abort) as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
