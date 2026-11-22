#!/usr/bin/env python3
"""Reachable cube grasps on the table, computed offline with the pick node's own planner.

    python3 -m so101_manipulation.reach_map --out DIR [--yaml config/reach.yaml] [--procs 4]
        [--max-tilt 45] [--pans -60 -45 ... 60] [--r-min 0.06] [--r-max 0.44] [--r-step 0.01] [--yaws 0 45]

writes DIR/reach_v2_<max tilt>.json (one row per pan / radius / cube yaw: ok, lean, roll, why) and
DIR/reach_v2_<max tilt>_summary.json (per pan: near and far limit, the lean needed there, what
stops the far end), and with --yaml the table the node publishes on ~/reach.

Radii are measured from base_link's origin (the dashboard's ring centre), pans about it as well;
the cube yaw is absolute (base_link). Runs niced, with at most --procs processes.

~/reach (std_msgs/Float32MultiArray, latched): one row per pan,
    [pan_deg, near_m, far_m, near_all_m, far_all_m]
near/far: the closest / farthest radius at which SOME tested cube yaw can be grasped (the band
between them is continuous at the scan step); *_all: where EVERY tested yaw can be grasped.
NaN when nothing at that pan is reachable.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np

COLUMNS = ["pan_deg", "near_m", "far_m", "near_all_m", "far_all_m"]

_planner = None


def classify(why: str) -> str:
    """What stopped a failed plan: per closing option, the reason it failed at the largest lean
    (from the planner's message), joined: e.g. "wrist_roll limit x2 / closing off horizontal"."""
    segs = [x.strip() for x in why.split(" | ")]
    tilted = [x[len("tilted:"):].strip() for x in segs if x.startswith("tilted:")] or segs
    cats = []
    for seg in tilted:
        if "needs roll" in seg:
            cats.append("wrist_roll limit")
        elif "off horizontal" in seg:
            cats.append("closing off horizontal")
        elif "lowest point" in seg:
            cats.append("table clearance")
        elif "at its limit" in seg:
            names = sorted({n for n in ("shoulder_lift", "elbow_flex", "wrist_flex") if f"{n} at its limit" in seg})
            cats.append("joint limit (" + ", ".join(names) + ")")
        elif "position off" in seg or "closing direction off" in seg:
            cats.append("out of reach (IK)")
        else:
            cats.append("other")
    out = []
    for c in sorted(set(cats), key=cats.index):
        n = cats.count(c)
        out.append(c + (f" x{n}" if n > 1 else ""))
    return " / ".join(out)


def _work(args):
    global _planner
    max_tilt, r, th, yaw, step, max_elev = args
    from .grasp import GraspPlanner
    from .kinematics import ArmModel
    if _planner is None or _planner.max_tilt_deg != max_tilt:
        _planner = GraspPlanner(ArmModel(), max_tilt_deg=max_tilt, allow_tilt=max_tilt > 0, tilt_step_deg=step,
                                max_close_elev_deg=max_elev)
    xy = np.array([r * math.cos(math.radians(th)), r * math.sin(math.radians(th))])
    t0 = time.time()
    plan, msg = _planner.plan_cube(xy, math.radians(yaw), math.radians(30.0))
    out = dict(tilt_max=max_tilt, r=round(r, 3), th=th, yaw=yaw, ok=plan is not None, s=round(time.time() - t0, 2))
    if plan is not None:
        out.update(tilt=round(plan.grasp.tilt_deg, 1), lean=round(plan.lean_deg, 1), roll=round(math.degrees(plan.roll)),
                   close_elev=round(plan.grasp.close_elev_deg, 1), tilted=bool(plan.tilted),
                   line="vertical" if plan.line is None else ("jaw axis" if plan.line[2] > -0.999 else "down"),
                   hover=round(plan.hover_h * 100, 1), grasp=round(plan.grasp_h * 100, 2), exit=plan.exit is not None)
    else:
        out["why"] = msg
        out["stop"] = classify(msg)
    return out


def summarize(rows, pans=None):
    """Per pan: near/far limits (some yaw / every yaw), the lean at those limits, and what stops the far end."""
    pans = sorted({x["th"] for x in rows}) if pans is None else pans
    yaws = sorted({x["yaw"] for x in rows})
    out = []
    for th in pans:
        rs = sorted({x["r"] for x in rows if x["th"] == th})
        ok_any = {r: [x for x in rows if x["th"] == th and x["r"] == r and x["ok"]] for r in rs}
        ok_all = {r: len(ok_any[r]) == len(yaws) for r in rs}

        def band(pred):
            good = [r for r in rs if pred(r)]
            if not good:
                return None, None
            # the contiguous band around the best radius (largest run of reachable radii)
            runs, cur = [], [good[0]]
            for a, b in zip(good, good[1:]):
                if rs.index(b) == rs.index(a) + 1:
                    cur.append(b)
                else:
                    runs.append(cur)
                    cur = [b]
            runs.append(cur)
            run = max(runs, key=len)
            return run[0], run[-1]
        near, far = band(lambda r: bool(ok_any[r]))
        near_all, far_all = band(lambda r: ok_all[r])
        row = dict(pan_deg=th, near_m=near, far_m=far, near_all_m=near_all, far_all_m=far_all)
        if far is not None:
            row["lean_at_far"] = {str(x["yaw"]): x["lean"] for x in ok_any[far]}
            row["lean_at_near"] = {str(x["yaw"]): x["lean"] for x in ok_any[near]}
            nxt = [x for x in rows if x["th"] == th and x["r"] > far and not x["ok"]]
            if nxt:
                beyond = min(x["r"] for x in nxt)
                row["far_stop"] = {str(x["yaw"]): classify(x.get("why", "")) for x in nxt if x["r"] == beyond}
            prv = [x for x in rows if x["th"] == th and x["r"] < near and not x["ok"]]
            if prv:
                before = max(x["r"] for x in prv)
                row["near_stop"] = {str(x["yaw"]): classify(x.get("why", "")) for x in prv if x["r"] == before}
        out.append(row)
    return out


def write_yaml(path, summary, meta):
    import yaml

    def f(v):
        return float("nan") if v is None else float(v)
    data = {"reach": {"columns": COLUMNS,
                      "rows": [[f(s["pan_deg"]), f(s["near_m"]), f(s["far_m"]), f(s["near_all_m"]), f(s["far_all_m"])]
                               for s in summary],
                      **meta}}
    with open(path, "w") as fh:
        fh.write("# Reachable cube grasps per pan (base_link, radius from its origin): written by\n"
                 "# python3 -m so101_manipulation.reach_map; published latched on /cube_pick_place/reach.\n")
        yaml.safe_dump(data, fh, sort_keys=False, default_flow_style=None)


def load_reach(path):
    import yaml
    with open(path) as fh:
        return yaml.safe_load(fh)["reach"]


def publish_reach(node, p):
    """Latched ~/reach (std_msgs/Float32MultiArray, rows [pan_deg, near_m, far_m, near_all_m, far_all_m])
    from the reach table; warns when it was computed with other planner settings. Returns the publisher."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension
    pub = node.create_publisher(Float32MultiArray, "~/reach",
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path = p.reach_file
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("so101_manipulation"), "config", "reach.yaml")
        except Exception:  # noqa: BLE001
            path = str(Path(__file__).resolve().parents[1] / "config" / "reach.yaml")
    try:
        r = load_reach(path)
    except Exception as e:  # noqa: BLE001
        node.get_logger().warn(f"no reach table ({e}): ~/reach not published")
        return pub
    for key, val in (("max_tilt_deg", p.max_tilt_deg), ("max_close_elev_deg", p.max_close_elev_deg),
                     ("roll_limit_deg", p.roll_limit_deg)):
        if key in r and abs(float(r[key]) - float(val)) > 1e-6:
            node.get_logger().warn(f"reach table computed with {key} {r[key]}, node uses {val}: "
                                   "rerun python3 -m so101_manipulation.reach_map")
    rows = r["rows"]
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(label="pan", size=len(rows), stride=len(rows) * len(COLUMNS)),
                      MultiArrayDimension(label=",".join(COLUMNS), size=len(COLUMNS), stride=len(COLUMNS))]
    msg.data = [float(v) for row in rows for v in row]
    pub.publish(msg)
    ok = [row for row in rows if row[1] == row[1]]
    if ok:
        node.get_logger().info(f"reach (some cube yaw): {min(x[1] for x in ok) * 100:.0f}-"
                               f"{max(x[2] for x in ok) * 100:.0f} cm over pans {rows[0][0]:+.0f}..{rows[-1][0]:+.0f} deg "
                               "(~/reach)")
    return pub


def main():
    from multiprocessing import Pool
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--yaml", default="")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--max-tilt", type=float, default=45.0)
    ap.add_argument("--tilt-step", type=float, default=5.0)
    ap.add_argument("--max-close-elev", type=float, default=20.0)
    ap.add_argument("--pans", type=float, nargs="+", default=[-60, -45, -30, -15, 0, 15, 30, 45, 60])
    ap.add_argument("--yaws", type=float, nargs="+", default=[0, 45])
    ap.add_argument("--r-min", type=float, default=0.06)
    ap.add_argument("--r-max", type=float, default=0.44)
    ap.add_argument("--r-step", type=float, default=0.01)
    a = ap.parse_args()
    try:
        os.nice(10)
    except OSError:
        pass
    radii = [round(x, 3) for x in np.arange(a.r_min, a.r_max + 1e-9, a.r_step)]
    jobs = [(a.max_tilt, r, th, y, a.tilt_step, a.max_close_elev) for th in a.pans for r in radii for y in a.yaws]
    t0 = time.time()
    with Pool(max(1, min(4, a.procs))) as pool:
        rows = pool.map(_work, jobs, chunksize=2)
    tag = f"{a.max_tilt:g}"
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, f"reach_v2_{tag}.json"), "w") as fh:
        json.dump(rows, fh, indent=0)
    summary = summarize(rows, a.pans)
    with open(os.path.join(a.out, f"reach_v2_{tag}_summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    if a.yaml:
        write_yaml(a.yaml, summary, {"max_tilt_deg": a.max_tilt, "tilt_step_deg": a.tilt_step,
                                     "max_close_elev_deg": a.max_close_elev, "roll_limit_deg": 90.0,
                                     "cube_yaws_deg": list(a.yaws), "r_step_m": a.r_step,
                                     "computed": time.strftime("%Y-%m-%d")})
    for s in summary:
        print(json.dumps(s))
    print(f"{len(rows)} plans in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
