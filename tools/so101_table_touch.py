#!/usr/bin/env python3
"""SO-101 table-touch calibration: let the hand touch the table, fit the joint zeros.

  probe  torque on. From the zero pose the arm visits poses just above the table and
         lowers the hand with elbow_flex or wrist_flex, slowly and at reduced torque,
         until it stops following its goal: that is the touch. The joint angles at
         every touch are saved.
  fit    at every touch the lowest point of the hand (from its meshes) lies on the
         table: fit the shoulder_lift, elbow_flex and wrist_flex zero offsets.

The base stands on the table, so the table is at the lowest point of the base meshes
(z = -2.4 mm in base_link). No camera and no board are needed, and the touches reach
folded poses that the hand-eye data never covered.

  python3 tools/so101_table_touch.py probe --dry-run     # plan only, no bus
  python3 tools/so101_table_touch.py probe --poses 20
  python3 tools/so101_table_touch.py fit

Opens the serial bus directly: never run it while so101_hardware is running.
"""

import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull

import feetech_bus as fb
import so101_hand_eye as he
from feetech_bus import FeetechError
from so101_calibrate import (DEFAULT_FILE, JOINTS, LOOP_HZ, TICKS_PER_DEG, Abort, LiveView, go_to_zero,
                             hold_all, load_calibration, move_joints, open_bus, require_servos, to_deg)

DATA_DIR = Path.home() / "so101_data" / "table_touch"
MESH_DIR = he.URDF.parents[1] / "meshes"

# Links that are checked against the table. A touch only counts on the hand links.
URDF_LINKS = ["upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]
CAMERA_LINKS = ["wrist_camera_mount_link", "wrist_camera_link"]
HAND_LINKS = {"wrist_link", "gripper_link", "moving_jaw_so101_v1_link"} | set(CAMERA_LINKS)

# probe
SAMPLE_DEG = {"shoulder_pan": (-30, 30), "shoulder_lift": (-60, 30), "elbow_flex": (-40, 85),
              "wrist_flex": (-90, 90), "wrist_roll": (-45, 45)}   # wider roll winds up the camera cable
DESCENT_JOINTS = ["elbow_flex", "wrist_flex"]   # alternated; the lift is too heavy at reduced torque
APPROACH_MM = 40.0       # the model puts the lowest point this high above the table before lowering
TRAVEL_CLEARANCE_MM = 30.0
RANGE_MARGIN_DEG = 5.0
DESCENT_DEG_S = 3.0
DESCENT_TORQUE = 60      # % for the lowering joint; the others hold at 100 %
MAX_DESCENT_DEG = 45.0   # the model was 39 mm too low in a folded pose: leave room for that
CONTACT_LAG_DEG = 1.5    # lag beyond the running baseline that counts as a touch
BASELINE_S = (1.5, 0.5)  # the baseline is the median lag between these many seconds ago

# fit
FIT_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
TABLE_Z = None           # set from the base meshes on first use


# --- mesh kinematics -----------------------------------------------------------

def stl_vertices(path):
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    return tri["v"].reshape(-1, 3).astype(np.float64)


def link_hull(urdf, link):
    """Convex hull vertices of a link's visual meshes in the link frame. The lowest point of
    a rigid body above a plane is always a hull vertex."""
    for l in ET.parse(urdf).getroot().iter("link"):
        if l.get("name") != link:
            continue
        pts = []
        for v in l.findall("visual"):
            mesh = v.find("geometry/mesh")
            scale = float((mesh.get("scale") or "1").split()[0])
            T = he.origin_transform(v)
            pts.append(stl_vertices(MESH_DIR / Path(mesh.get("filename")).name) * scale @ T[:3, :3].T + T[:3, 3])
        P = np.vstack(pts)
        return P[ConvexHull(P).vertices]
    raise KeyError(link)


class Model:
    def __init__(self, table_mm=0.0):
        """table_mm: table height relative to the lowest point of the base (negative: below)."""
        gripper = he.load_chain(he.URDF, he.BASE, "gripper_link")
        self.links = [(l, he.load_chain(he.URDF, he.BASE, l), link_hull(he.URDF, l)) for l in URDF_LINKS]
        self.links += [(l, gripper + he.load_chain(he.CAMERA_XACRO, "${parent}", l), link_hull(he.CAMERA_XACRO, l))
                       for l in CAMERA_LINKS]
        self.table_z = link_hull(he.URDF, "base_link")[:, 2].min() + table_mm / 1000

    def lowest(self, q_deg):
        """(link, height above the table [m], point in base_link) of the lowest point."""
        q = {n: np.radians(v) for n, v in q_deg.items()}
        best = None
        for name, chain, P in self.links:
            T = he.forward(chain, q)
            z = P @ T[2, :3] + T[2, 3]
            i = int(np.argmin(z))
            if best is None or z[i] < best[1]:
                best = (name, z[i], T[:3, :3] @ P[i] + T[:3, 3])
        return best[0], best[1] - self.table_z, best[2]

    def height(self, q_deg):
        return self.lowest(q_deg)[1]


# --- planning --------------------------------------------------------------------

def joint_limits(cal, n):
    lo, hi = sorted((to_deg(cal[n]["range_min"], cal[n]), to_deg(cal[n]["range_max"], cal[n])))
    return lo + RANGE_MARGIN_DEG, hi - RANGE_MARGIN_DEG


def plan_probe(model, cal, q, joint):
    """Approach angle of `joint` (lowest point APPROACH_MM above the table), the direction
    that lowers the hand, and the predicted touch angle; None if this pose does not work."""
    lo, hi = joint_limits(cal, joint)
    for sign in (-1, 1):
        a = q[joint]
        h = model.height({**q, joint: a})
        if h < APPROACH_MM / 1000:
            return None
        # walk towards the table until the approach height, then on to the touch
        approach = None
        while lo <= a <= hi:
            h_next = model.height({**q, joint: a + sign})
            if h_next >= h:        # not going down this way
                break
            a, h = a + sign, h_next
            if approach is None and h <= APPROACH_MM / 1000:
                approach = a
            if h <= 0:
                link = model.lowest({**q, joint: a})[0]
                return (approach, sign, a) if approach is not None and link in HAND_LINKS else None
    return None


def path_clearance(model, a, b, steps=30):
    return min(model.height({n: a[n] + (b[n] - a[n]) * s for n in a}) for s in np.linspace(0, 1, steps))


def plan_poses(model, cal, count, rng, start):
    """Random probe poses; each one reachable from the previous by a straight joint move
    that keeps TRAVEL_CLEARANCE_MM, or else via the zero pose."""
    zero = {n: 0.0 for n in start}
    plans, prev, tries = [], start, 0
    while len(plans) < count and tries < 200 * count:
        tries += 1
        joint = DESCENT_JOINTS[len(plans) % len(DESCENT_JOINTS)]
        q = dict(start)
        for n, (lo, hi) in SAMPLE_DEG.items():
            jl, jh = joint_limits(cal, n)
            q[n] = rng.uniform(max(lo, jl), min(hi, jh))
        p = plan_probe(model, cal, q, joint)
        if p is None:
            continue
        approach, sign, touch = p
        q[joint] = approach
        clear = TRAVEL_CLEARANCE_MM / 1000
        if path_clearance(model, prev, q) >= clear:
            via = False
        elif path_clearance(model, prev, zero) >= clear and path_clearance(model, zero, q) >= clear:
            via = True
        else:
            continue
        plans.append({"q": q, "joint": joint, "sign": sign, "touch": touch, "via_zero": via})
        prev = q
    return plans


# --- probing ---------------------------------------------------------------------

def read_ticks(bus, joints):
    return {j.name: bus.read_position(j.id) for j in joints}


def freeze(bus, joints):
    for j in joints:
        bus.write_u16(j.id, fb.GOAL_POSITION, bus.read_position(j.id))


def descend(bus, j, c, start, sign, view):
    """Lower the hand with joint j from `start` [deg] until it stops following. Returns the
    touch (True) or why there was none."""
    lo, hi = joint_limits({j.name: c}, j.name)
    lags = []  # (time, lag)
    t0 = time.monotonic()
    bus.write_u16(j.id, fb.TORQUE_LIMIT, DESCENT_TORQUE * 10)
    try:
        while True:
            now = time.monotonic() - t0
            goal = start + sign * DESCENT_DEG_S * now
            if not lo <= goal <= hi or abs(goal - start) > MAX_DESCENT_DEG:
                return "no touch within range"
            bus.write_u16(j.id, fb.GOAL_POSITION, round(c["zero"] + c["direction"] * goal * TICKS_PER_DEG))
            time.sleep(1 / LOOP_HZ)
            try:
                pos = to_deg(bus.read_position(j.id), c)
            except FeetechError:
                continue
            lag = sign * (goal - pos)  # > 0: behind the goal
            lags.append((now, lag))
            base = [l for t, l in lags if now - BASELINE_S[0] <= t <= now - BASELINE_S[1]]
            base = float(np.median(base)) if now > BASELINE_S[0] else None
            view.show([f"  lowering {j.name}: goal {goal:+7.1f}°  at {pos:+7.1f}°  lag {lag:+5.1f}°"
                       + (f"  (baseline {base:+.1f}°)" if base is not None else "")])
            if base is not None and lag - base > CONTACT_LAG_DEG:
                return True
    finally:
        freeze(bus, [j])
        bus.write_u16(j.id, fb.TORQUE_LIMIT, 1000)


def to_target(cal, q_deg):
    return {n: round(cal[n]["zero"] + cal[n]["direction"] * v * TICKS_PER_DEG) for n, v in q_deg.items()}


def cmd_probe(args):
    cal = load_calibration(args.file)
    model = Model(args.table_mm)
    joints = [j for j in JOINTS if j.name in cal]
    names = [n for n in SAMPLE_DEG]
    zero = {n: 0.0 for n in names + ["gripper"]}  # the gripper stays at its zero (go_to_zero)
    plans = plan_poses(model, cal, args.poses, np.random.default_rng(args.seed), zero)
    print(f"{len(plans)} probe poses planned (seed {args.seed})")
    for i, p in enumerate(plans, 1):
        print(f"  {i:2d} " + " ".join(f"{n.split('_')[1]} {p['q'][n]:+6.1f}" for n in names)
              + f"  lower {p['joint']} {p['sign']:+d}, model touch at {p['touch']:+.0f}°" + ("  (via zero)" if p["via_zero"] else ""))
    if args.dry_run or not plans:
        return 0

    run = DATA_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
    run.mkdir(parents=True)
    out = open(run / "touches.jsonl", "w")
    with open_bus(args) as bus:
        require_servos(bus, joints)
        print("The arm goes to zero (elbow first), then probes. Keep the table under the hand clear.")
        print("Ctrl+C freezes the arm where it is (torque stays on).")
        input("Enter to start... ")
        view = LiveView()
        try:
            hold_all(bus, joints)
            if go_to_zero(bus, cal, joints):
                raise Abort("stuck on the way to zero")
            arm = [j for j in joints if j.name in names]
            touches = 0
            for i, p in enumerate(plans, 1):
                print(f"\n[{i}/{len(plans)}] lower {p['joint']}")
                legs = ([zero] if p["via_zero"] else []) + [p["q"]]
                for leg in legs:
                    stuck = move_joints(bus, cal, arm, to_target(cal, leg))
                    if stuck:
                        raise Abort(f"stuck while moving ({', '.join(stuck)}): something is in the way")
                j = next(j for j in arm if j.name == p["joint"])
                result = descend(bus, j, cal[j.name], p["q"][j.name], p["sign"], view)
                time.sleep(0.3)
                ticks = read_ticks(bus, joints)
                if result is True:
                    q = {n: to_deg(t, cal[n]) for n, t in ticks.items()}
                    link, h, _ = model.lowest(q)
                    touches += 1
                    print(f"  touch: {j.name} {q[j.name]:+.1f}° (model said {p['touch']:+.0f}°), "
                          f"model puts {link} {h * 1000:+.1f} mm off the table")
                    out.write(json.dumps({"ticks": ticks, "joint": j.name, "sign": p["sign"],
                                          "model_mm": round(h * 1000, 1), "link": link}) + "\n")
                    out.flush()
                else:
                    print(f"  {result}")
                # back up to the approach pose before travelling on
                stuck = move_joints(bus, cal, arm, to_target(cal, p["q"]))
                if stuck:
                    raise Abort(f"could not back up from the table ({', '.join(stuck)})")
            print(f"\n{touches} touches saved in {run}")
            move_joints(bus, cal, arm, to_target(cal, zero))
            print("at zero, holding.")
        except KeyboardInterrupt:
            freeze(bus, joints)
            print("\ninterrupted: frozen in place, holding.")
        except Abort as e:
            freeze(bus, joints)
            print(f"\n{e}. Frozen in place, holding.")
        finally:
            out.close()
    print("release: python3 tools/so101_calibrate.py off  (support the arm first)")
    print(f"then: python3 tools/so101_table_touch.py fit {run}")
    return 0


# --- fitting ---------------------------------------------------------------------

def load_touches(folders):
    touches = []
    for f in folders:
        for line in (f / "touches.jsonl").read_text().splitlines():
            if line.strip():
                touches.append(json.loads(line))
    return touches


def residuals(model, qs, x):
    """Height of the lowest point above the table [mm] at every touch, with zero offsets
    x[:3] [deg] and, if x has a fourth entry, the table height [mm]."""
    d = dict(zip(FIT_JOINTS, x))
    table = x[len(FIT_JOINTS)] if len(x) > len(FIT_JOINTS) else 0.0
    return np.array([model.height({n: v + d.get(n, 0.0) for n, v in q.items()}) * 1000 - table for q in qs])


def fit_offsets(model, qs, free_table=False):
    n = len(FIT_JOINTS) + free_table
    r = least_squares(lambda x: residuals(model, qs, x), np.zeros(n), diff_step=1e-3)
    s2 = float(r.fun @ r.fun) / max(1, len(qs) - n)
    try:
        sd = np.sqrt(np.diag(np.linalg.inv(r.jac.T @ r.jac) * s2))
    except np.linalg.LinAlgError:
        sd = np.full(n, np.inf)
    return r.x, sd, r.fun


def cmd_fit(args):
    cal = load_calibration(args.file)
    folders = args.runs or sorted(p for p in DATA_DIR.iterdir() if (p / "touches.jsonl").exists())[-1:]
    touches = load_touches(folders)
    print(f"{len(touches)} touches from {', '.join(f.name for f in folders)}")
    n_par = len(FIT_JOINTS) + args.free_table
    if len(touches) < 2 * n_par:
        raise Abort("too few touches to fit")
    model = Model()
    qs = [{n: to_deg(t, cal[n]) for n, t in tc["ticks"].items()} for tc in touches]
    before = residuals(model, qs, np.zeros(len(FIT_JOINTS)))
    x, sd, after = fit_offsets(model, qs, args.free_table)
    print(f"\nlowest point above the table: RMS {np.sqrt(np.mean(before ** 2)):.1f} mm before, "
          f"{np.sqrt(np.mean(after ** 2)):.1f} mm after the fit")
    for i, (tc, b, a) in enumerate(zip(touches, before, after), 1):
        q = qs[i - 1]
        print(f"  {i:2d} lift {q['shoulder_lift']:+6.1f} elbow {q['elbow_flex']:+6.1f} wflex {q['wrist_flex']:+6.1f}"
              f"  {tc['link']:<26} {b:+6.1f} -> {a:+5.1f} mm")
    print("\nzero offsets (true angle = calibrated angle + offset):")
    for n, v, s in zip(FIT_JOINTS, x, sd):
        print(f"  {n:<14} {v:+6.2f} ± {s:.2f}°   zero tick {cal[n]['zero']} -> "
              f"{round(cal[n]['zero'] - cal[n]['direction'] * v * TICKS_PER_DEG)}")
    if args.free_table:
        print(f"  table height   {x[-1]:+6.1f} ± {sd[-1]:.1f} mm relative to the lowest point of the base")
    halves = [qs[0::2], qs[1::2]]
    if min(map(len, halves)) >= n_par + 1:
        print("split halves (even / odd touches):")
        hx = [fit_offsets(model, h, args.free_table)[0] for h in halves]
        for k, n in enumerate(FIT_JOINTS + (["table [mm]"] if args.free_table else [])):
            print(f"  {n:<14} {hx[0][k]:+6.2f}  {hx[1][k]:+6.2f}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"calibration file (default: {DEFAULT_FILE})")
    sub = parser.add_subparsers(dest="cmd", required=True)
    probe = sub.add_parser("probe")
    probe.add_argument("--poses", type=int, default=20)
    probe.add_argument("--seed", type=int, default=0)
    probe.add_argument("--dry-run", action="store_true", help="only plan and print the poses")
    probe.add_argument("--table-mm", type=float, default=0.0,
                       help="where the table is for planning, relative to the lowest point of the base (negative: below)")
    fit = sub.add_parser("fit")
    fit.add_argument("runs", nargs="*", type=Path, help=f"run folders (default: the latest in {DATA_DIR})")
    fit.add_argument("--free-table", action="store_true", help="fit the table height as well")
    args = parser.parse_args()
    try:
        return {"probe": cmd_probe, "fit": cmd_fit}[args.cmd](args)
    except (FeetechError, Abort) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
