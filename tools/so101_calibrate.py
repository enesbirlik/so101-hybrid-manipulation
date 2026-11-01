#!/usr/bin/env python3
"""SO-101 joint calibration: homing offsets, ranges and zero ticks for the URDF.

  scan       ping the six servos, show voltage / temperature / registers
  live       live encoder view (ticks, and degrees/radians once calibrated)
  calibrate  joint by joint, torque off: move it to both ends, hold it at its
             zero, press Enter -> EEPROM offset + limits, saved, joint locked
  check      startup sanity check of the servos against the calibration file
  zero       move slowly to the calibrated zero pose and hold it there
  off        switch torque off on every joint (support the arm first)

Each joint is written only after you confirm it, and the calibration file is
saved right after, so an aborted run never leaves servos and file out of sync.

The ST3215 encoder is absolute within one turn and the homing offset lives in
EEPROM, so calibration is a one-time step. Redo it only after a horn was taken
off or a servo was swapped; `check` tells you when that is needed.

Opens the serial bus directly: never run it while so101_hardware is running.
"""

import argparse
import math
import select
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import yaml

import feetech_bus as fb
from feetech_bus import FeetechBus, FeetechError

TICKS_PER_DEG = fb.TICKS_PER_TURN / 360.0
HALF_TURN = fb.TICKS_PER_TURN // 2

DEFAULT_FILE = Path(__file__).resolve().parents[1] / "so101_bringup" / "config" / "so101_calibration.yaml"

LIMIT_MARGIN_DEG = 2.0   # servo position limits sit this far inside the measured range
LOOP_HZ = 50
STALL_WINDOW = 0.5       # s; a joint that gets no closer to its goal within this is stuck

# Moves (zero, teleop): the joint with the longest way to go moves at MOVE_SPEED,
# the others slower so that all of them arrive together.
MOVE_SPEED = 200         # step/s, about 18 deg/s
MOVE_ACC = 10            # 100 step/s^2
MOVE_TORQUE = 100        # % of max torque; 70 % could not hold the arm horizontal
# Going to zero from the folded rest pose: unfold the elbow first, then the lift, then the
# rest. Lifting first with the elbow folded swings the hand down: the jaw hit the table at
# lift -18 deg (2026-10-29). URDF FK: elbow first keeps every link >= 146 mm above the table.
ZERO_STAGES = [{"elbow_flex"}, {"shoulder_lift"}, {"shoulder_pan", "wrist_flex", "wrist_roll", "gripper"}]


@dataclass
class Joint:
    id: int
    name: str
    urdf_lower: float     # deg, so101_new_calib.urdf
    urdf_upper: float     # deg
    zero_from: str = ""   # "lower"/"upper": that hard stop sets the zero, the Enter pose is ignored
    stop_deg: float = None  # angle of that hard stop (default: the URDF limit)


JOINTS = [
    Joint(1, "shoulder_pan", -110.0, 110.0),
    Joint(2, "shoulder_lift", -100.0, 100.0),
    # Folded onto the upper arm. The URDF limit (+96.8) is 5 deg wider than where its meshes touch.
    Joint(3, "elbow_flex", -96.8, 96.8, zero_from="upper", stop_deg=91.8),
    Joint(4, "wrist_flex", -95.0, 95.0),
    Joint(5, "wrist_roll", -157.2, 162.8),
    Joint(6, "gripper", -10.0, 100.0, zero_from="lower"),  # closed = -10 deg
]


class Abort(Exception):
    pass


class Tracker:
    """Follows one joint across the 0/4095 wrap: `u` counts ticks continuously."""

    def __init__(self, pos):
        self.last = self.u = self.lo = self.hi = pos

    def update(self, pos):
        self.u += wrap(pos - self.last)
        self.last = pos
        self.lo, self.hi = min(self.lo, self.u), max(self.hi, self.u)


def wrap(ticks):
    """Map a tick difference into [-2048, 2047]."""
    return (ticks + HALF_TURN) % fb.TICKS_PER_TURN - HALF_TURN


# --- terminal helpers --------------------------------------------------------

class LiveView:
    """Redraws a block of lines in place."""

    def __init__(self):
        self.lines = 0

    def show(self, lines):
        out = f"\x1b[{self.lines}F" if self.lines else ""
        out += "".join(f"\x1b[2K{line}\n" for line in lines)
        sys.stdout.write(out)
        sys.stdout.flush()
        self.lines = len(lines)


def enter_pressed():
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def ask(prompt, choices, default):
    while True:
        answer = input(prompt).strip().lower() or default
        if answer in choices:
            return answer


def deg(ticks):
    return ticks / TICKS_PER_DEG


def load_calibration(path):
    if not path.exists():
        return {}
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get("joints", {})


def save_calibration(path, cal, port):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# SO-101 follower calibration, written by tools/so101_calibrate.py",
        f"# date: {datetime.now().isoformat(timespec='seconds')}",
        f"# port: {port}",
        "#",
        "# All ticks are servo Present_Position values with homing_offset applied",
        "# (the offset is stored in each servo's EEPROM, address 31).",
        "#   joint_rad = direction * (ticks - zero) * 2*pi / 4096",
        "# zero is the pose held at Enter during calibration (the URDF zero pose);",
        "# for elbow_flex and the gripper it comes from a hard stop (folded = +91.8 deg,",
        "# closed = -10 deg).",
        "# direction is not measured: verify it against RViz before trusting it.",
        "joints:",
    ]
    for j in JOINTS:
        if j.name not in cal:
            continue
        c = cal[j.name]
        lines += [
            f"  {j.name}:",
            f"    id: {c['id']}",
            f"    homing_offset: {c['homing_offset']}",
            f"    range_min: {c['range_min']}",
            f"    range_max: {c['range_max']}",
            f"    zero: {c['zero']}",
            f"    direction: {c['direction']}",
        ]
    path.write_text("\n".join(lines) + "\n")


def to_deg(ticks, c):
    return c["direction"] * deg(ticks - c["zero"])


# --- bus helpers -------------------------------------------------------------

def open_bus(args):
    port = args.port or fb.find_port()
    print(f"port: {port}")
    return FeetechBus(port)


def require_servos(bus, joints):
    missing = [j for j in joints if not bus.ping(j.id)]
    if missing:
        raise Abort("no reply from: " + ", ".join(f"{j.name} (id {j.id})" for j in missing))


def write_eeprom(bus, servo_id, offset, lo, hi):
    with bus.eeprom_unlocked(servo_id):
        bus.write_homing_offset(servo_id, offset)
        bus.write_u16(servo_id, fb.MIN_POSITION_LIMIT, lo)
        bus.write_u16(servo_id, fb.MAX_POSITION_LIMIT, hi)


# --- commands ----------------------------------------------------------------

def cmd_scan(args):
    with open_bus(args) as bus:
        print(f"{'joint':<14}{'id':>3}{'pos':>7}{'offset':>8}{'min':>6}{'max':>6}{'mode':>6}{'torque':>8}{'V':>7}{'C':>5}")
        for j in JOINTS:
            if not bus.ping(j.id):
                print(f"{j.name:<14}{j.id:>3}  no reply")
                continue
            pos, _, _, volt, temp = bus.read_state(j.id)
            print(f"{j.name:<14}{j.id:>3}{pos:>7}{bus.read_homing_offset(j.id):>8}"
                  f"{bus.read_u16(j.id, fb.MIN_POSITION_LIMIT):>6}{bus.read_u16(j.id, fb.MAX_POSITION_LIMIT):>6}"
                  f"{bus.read_u8(j.id, fb.OPERATING_MODE):>6}{bus.read_u8(j.id, fb.TORQUE_ENABLE):>8}"
                  f"{volt:>7.1f}{temp:>5}")


def cmd_live(args):
    cal = load_calibration(args.file)
    with open_bus(args) as bus:
        require_servos(bus, JOINTS)
        if args.torque_off:
            for j in JOINTS:
                bus.torque(j.id, False)
        if not cal:
            print(f"no calibration in {args.file}: showing raw ticks only")
        print("Ctrl+C to quit")
        view = LiveView()
        t_end = time.monotonic() + args.duration if args.duration else math.inf
        try:
            while time.monotonic() < t_end:
                lines = [f"{'joint':<14}{'ticks':>6}{'deg':>9}{'rad':>9}{'load %':>8}{'C':>4}  range"]
                for j in JOINTS:
                    pos, _, load, _, temp = bus.read_state(j.id)
                    c = cal.get(j.name)
                    if c:
                        d = to_deg(pos, c)
                        inside = c["range_min"] <= pos <= c["range_max"]
                        cols = f"{d:>9.1f}{math.radians(d):>9.3f}"
                        flag = "ok" if inside else "OUTSIDE"
                    else:
                        cols, flag = f"{'-':>9}{'-':>9}", ""
                    lines.append(f"{j.name:<14}{pos:>6}{cols}{load / 10:>8.1f}{temp:>4}  {flag}")
                view.show(lines)
                time.sleep(1 / 20)
        except KeyboardInterrupt:
            print()


def cmd_calibrate(args):
    joints = [j for j in JOINTS if not args.joints or j.name in args.joints]
    cal = load_calibration(args.file)

    with open_bus(args) as bus:
        require_servos(bus, JOINTS)
        for j in joints:
            bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
        print("\nJoint by joint: move it to both ends, hold it at its zero, press Enter.")
        print("Each confirmed joint is written to EEPROM, saved and locked (torque on).")
        print("Ctrl+C: torque off everywhere; joints already written stay saved.")
        locked = []
        try:
            for n, j in enumerate(joints, 1):
                print(f"\n[{n}/{len(joints)}] {j.name} (id {j.id}), URDF {j.urdf_lower:+.1f} .. {j.urdf_upper:+.1f} deg")
                while True:
                    plan = plan_joint(bus, j, track_joint(bus, j, locked))
                    if plan and ask("  write to EEPROM and lock? [Y/n = redo]: ", "yn", "y") == "y":
                        break
                # Ctrl+C must not stop us half way through the EEPROM writes.
                sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
                try:
                    write_joint(bus, j, plan, cal)
                    save_calibration(args.file, cal, bus.port)
                    bus.write_u16(j.id, fb.TORQUE_LIMIT, 1000)
                    bus.torque(j.id, True)
                finally:
                    signal.signal(signal.SIGINT, sigint)
                locked.append(j)
                print(f"  written, verified, saved to {args.file.name}, locked.")
            input("\nAll joints done. Support the arm, then press Enter to switch torque off... ")
        except KeyboardInterrupt:
            print("\ninterrupted. The joint in progress was not changed.")
        finally:
            for j in JOINTS:
                bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
            print("torque off.")
    print("next: python3 tools/so101_calibrate.py check")


def track_joint(bus, j, locked):
    """Track one joint across the encoder wrap until Enter; the Enter pose is its zero."""
    if j.zero_from:
        limit = stop_angle(j)
        print(f"  Move it by hand to BOTH ends and press Enter (the {j.zero_from} hard stop becomes {limit:+.1f}°).")
    else:
        print("  Move it by hand to BOTH ends, then hold it at its zero and press Enter.")
    t = Tracker(bus.read_position(j.id))
    view = LiveView()
    while not enter_pressed():
        try:
            t.update(bus.read_position(j.id))
        except FeetechError:
            pass  # a missed reply; the next sample catches up
        lines = [f"  range {deg(t.hi - t.lo):6.1f}° (URDF {j.urdf_upper - j.urdf_lower:.1f}°)   "
                 f"with zero here: {deg(t.lo - t.u):+7.1f}° .. {deg(t.hi - t.u):+7.1f}° "
                 f"(URDF {j.urdf_lower:+.1f}° .. {j.urdf_upper:+.1f}°)"]
        for k in locked:
            try:
                _, _, load, _, temp = bus.read_state(k.id)
                lines.append(f"    locked {k.name:<14} load {load / 10:+6.1f} %  {temp} C")
            except FeetechError:
                lines.append(f"    locked {k.name:<14} no reply")
        view.show(lines)
        time.sleep(1 / LOOP_HZ)
    return t


def stop_angle(j):
    if j.stop_deg is not None:
        return j.stop_deg
    return j.urdf_lower if j.zero_from == "lower" else j.urdf_upper


def plan_joint(bus, j, t):
    """Homing offset and range. The offset makes the Enter pose read ~2048 (the middle of
    the range for joints whose zero comes from a hard stop), keeping both ends off the wrap."""
    urdf_span = (j.urdf_upper - j.urdf_lower) * TICKS_PER_DEG
    if t.hi - t.lo < 0.3 * urdf_span:
        print(f"  ! range only {deg(t.hi - t.lo):.1f}°: move it to both ends first")
        return None
    ref = (t.lo + t.hi) // 2 if j.zero_from else t.u
    old_offset = bus.read_homing_offset(j.id)
    # present = (raw - offset) mod 4096, so lowering the offset by s raises every reading by s.
    offset = max(wrap(old_offset + ref - HALF_TURN), -2047)  # 11-bit magnitude
    shift = old_offset - offset
    zero = (ref + shift) % fb.TICKS_PER_TURN
    lo, hi = zero + (t.lo - ref), zero + (t.hi - ref)
    if j.zero_from == "lower":
        zero = lo - round(stop_angle(j) * TICKS_PER_DEG)
    elif j.zero_from == "upper":
        zero = hi - round(stop_angle(j) * TICKS_PER_DEG)
    print(f"  zero here, range {deg(lo - zero):+.1f}° .. {deg(hi - zero):+.1f}° "
          f"(URDF {j.urdf_lower:+.1f}° .. {j.urdf_upper:+.1f}°)")
    if lo < 0 or hi >= fb.TICKS_PER_TURN:
        print("  ! an end is more than 180° from this zero (encoder wrap): pick a zero nearer the middle")
        return None
    if hi - lo < 0.8 * urdf_span:
        print("  ! much less than the URDF range: moved to both ends?")
    return offset, shift, zero, lo, hi


def write_joint(bus, j, plan, cal):
    offset, shift, zero, lo, hi = plan
    margin = round(LIMIT_MARGIN_DEG * TICKS_PER_DEG)
    old = (bus.read_homing_offset(j.id), bus.read_u16(j.id, fb.MIN_POSITION_LIMIT),
           bus.read_u16(j.id, fb.MAX_POSITION_LIMIT))
    expected = (bus.read_position(j.id) + shift) % fb.TICKS_PER_TURN
    if hi - lo > 2 * margin:
        write_eeprom(bus, j.id, offset, lo + margin, hi - margin)
    else:
        write_eeprom(bus, j.id, offset, 0, fb.TICKS_PER_TURN - 1)
    if abs(wrap(bus.read_position(j.id) - expected)) > 30:
        write_eeprom(bus, j.id, *old)
        raise Abort(f"{j.name}: readback mismatch, old servo values restored")
    cal[j.name] = {
        "id": j.id,
        "homing_offset": offset,
        "range_min": lo,
        "range_max": hi,
        "zero": zero,
        "direction": cal.get(j.name, {}).get("direction", 1),
    }


def cmd_check(args):
    cal = load_calibration(args.file)
    if not cal:
        print(f"no calibration in {args.file}")
        return 1
    ok = True
    with open_bus(args) as bus:
        for j in JOINTS:
            c = cal.get(j.name)
            if not c:
                print(f"  {j.name:<14} not calibrated")
                ok = False
                continue
            if not bus.ping(c["id"]):
                print(f"  {j.name:<14} FAIL no reply from id {c['id']}")
                ok = False
                continue
            offset = bus.read_homing_offset(c["id"])
            pos = bus.read_position(c["id"])
            problems = []
            if offset != c["homing_offset"]:
                problems.append(f"homing offset {offset} != {c['homing_offset']} (servo reset or swapped?)")
            if not c["range_min"] - 20 <= pos <= c["range_max"] + 20:
                problems.append(f"position {pos} outside {c['range_min']} .. {c['range_max']}")
            ok &= not problems
            status = "FAIL " + "; ".join(problems) if problems else f"ok  {to_deg(pos, c):+7.1f} deg"
            print(f"  {j.name:<14} {status}")
    print("calibration OK" if ok else "recalibrate: python3 tools/so101_calibrate.py calibrate")
    return 0 if ok else 1


def hold_all(bus, joints):
    """Torque on every joint where it is now, with the slow move profile and full torque."""
    for j in joints:
        bus.write_u8(j.id, fb.ACCELERATION, MOVE_ACC)
        bus.write_u16(j.id, fb.GOAL_SPEED, MOVE_SPEED)
        bus.torque(j.id, True)
        bus.write_u16(j.id, fb.TORQUE_LIMIT, MOVE_TORQUE * 10)


def move_joints(bus, cal, joints, target):
    """Move `joints` to `target` ticks, arriving together. Freezes all of them if one gets stuck.

    Returns the names of the stuck joints ([] when every joint arrived, ["timeout"] on timeout).
    """
    start = {j.name: bus.read_position(j.id) for j in joints}
    longest = max(abs(start[j.name] - target[j.name]) for j in joints) or 1
    for j in joints:
        speed = max(20, round(MOVE_SPEED * abs(start[j.name] - target[j.name]) / longest))
        bus.write_u16(j.id, fb.GOAL_SPEED, speed)
        bus.write_u16(j.id, fb.GOAL_POSITION, target[j.name])

    view = LiveView()
    t0 = time.monotonic()
    t_end = t0 + longest / MOVE_SPEED + 4.0
    history = []  # (time, positions), last STALL_WINDOW seconds
    stuck = []
    while True:
        now = time.monotonic()
        pos = {j.name: bus.read_position(j.id) for j in joints}
        history = [h for h in history if now - h[0] <= STALL_WINDOW] + [(now, pos)]
        view.show([f"  {j.name:<14}{to_deg(pos[j.name], cal[j.name]):+7.1f} deg" for j in joints])
        if all(abs(pos[n] - target[n]) <= 25 for n in pos):
            break
        if now - t0 > 1.0 and now - history[0][0] >= STALL_WINDOW * 0.9:
            # far from the goal and no longer getting closer: blocked or overpowered
            stuck = [n for n in pos if abs(pos[n] - target[n]) > 25
                     and abs(history[0][1][n] - target[n]) - abs(pos[n] - target[n]) < 5]
        if stuck or now > t_end:
            stuck = stuck or ["timeout"]
            break
        time.sleep(1 / LOOP_HZ)
    for j in joints:  # restore the normal speed; freeze in place if we did not arrive
        bus.write_u16(j.id, fb.GOAL_SPEED, MOVE_SPEED)
        if stuck:
            bus.write_u16(j.id, fb.GOAL_POSITION, bus.read_position(j.id))
    return stuck


def go_to_zero(bus, cal, joints):
    """Stage by stage so the folded arm unfolds without pushing itself into the table."""
    for stage in ZERO_STAGES:
        group = [j for j in joints if j.name in stage]
        if not group:
            continue
        print(f"  -> {', '.join(j.name for j in group)}")
        stuck = move_joints(bus, cal, group, {j.name: cal[j.name]["zero"] for j in group})
        if stuck:
            return stuck
    return []


def cmd_zero(args):
    cal = load_calibration(args.file)
    joints = [j for j in JOINTS if j.name in cal]
    if not joints:
        raise Abort(f"no calibration in {args.file}")
    with open_bus(args) as bus:
        require_servos(bus, joints)
        try:
            hold_all(bus, joints)
            stuck = go_to_zero(bus, cal, joints)
        except KeyboardInterrupt:
            for j in joints:
                bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
            print("\ninterrupted, torque off.")
            return 1
    if stuck:
        print(f"stopped, not moving towards zero: {', '.join(stuck)}. Frozen in place, holding.")
    else:
        print("at zero, holding.")
    print("release: python3 tools/so101_calibrate.py off  (support the arm first)")
    return 1 if stuck else 0


def cmd_off(args):
    with open_bus(args) as bus:
        for j in JOINTS:
            bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
    print("torque off.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"calibration file (default: {DEFAULT_FILE})")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    live = sub.add_parser("live")
    live.add_argument("--torque-off", action="store_true", help="switch torque off first so you can move the arm")
    live.add_argument("--duration", type=float, help="stop after this many seconds")
    calib = sub.add_parser("calibrate")
    calib.add_argument("--joints", nargs="+", choices=[j.name for j in JOINTS], help="only these joints")
    sub.add_parser("check")
    sub.add_parser("zero")
    sub.add_parser("off")
    args = parser.parse_args()

    commands = {"scan": cmd_scan, "live": cmd_live, "calibrate": cmd_calibrate, "check": cmd_check,
                "zero": cmd_zero, "off": cmd_off}
    try:
        return commands[args.cmd](args)
    except (FeetechError, Abort) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
