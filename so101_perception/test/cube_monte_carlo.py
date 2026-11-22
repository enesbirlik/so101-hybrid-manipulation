#!/usr/bin/env python3
"""Monte Carlo accuracy of the cube detector on imperfect paper cubes (see cube_sim.py).

    python3 so101_perception/test/cube_monte_carlo.py [--n 300] [--seed 1] [--perfect]
            [--module path/to/other/cube_detection.py]   # e.g. an older version, for comparison
            [--views 6]    # also: each cube seen from this many wrist poses, fused by the tracker

Prints the detection rate and the centre (xy) and yaw (mod 90) errors: p50 / p95 / max.
"""

import argparse
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import cube_sim as sim  # noqa: E402


def load_module(path):
    spec = importlib.util.spec_from_file_location("cube_detection_under_test", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def estimate(module, detector, gray, K, D, R_wc, t_wc, table_centre_z):
    """-> {cube: (centre, yaw)} with the module's own API (new: estimate_cubes, old: cube_estimates)."""
    detections = detector.detect(gray)
    if hasattr(module, "estimate_cubes"):
        cubes = module.estimate_cubes(detections, K, D, R_wc, t_wc, table_centre_z=table_centre_z)
    else:
        poses = module.marker_poses(detections, K, D)
        cubes = module.cube_estimates(poses, R_wc, t_wc, table_centre_z=table_centre_z)
    return {e.cube: e for e in cubes}


def run(n=300, seed=1, imp=None, module=None, verbose=False):
    """-> dict of arrays: xy error (m), yaw error (rad, folded), faces used, per detected sample;
    plus the number of samples in which the cube was visible enough to be expected."""
    imp = sim.Imperfections() if imp is None else imp
    module = module or sim.cd
    detector = module.MarkerDetector()
    K, D = sim.camera()
    rng = np.random.default_rng(seed)
    xy, yaw, faces, missed, wrong = [], [], [], 0, 0
    for i in range(n):
        cube, R_wc, t_wc = sim.random_scene(rng, imp)
        gray = sim.render([cube], R_wc, t_wc, K, D, rng=rng)
        found = estimate(module, detector, gray, K, D, R_wc, t_wc, sim.EDGE / 2)
        wrong += len(set(found) - {cube.k})
        e = found.get(cube.k)
        if e is None:
            missed += 1
            continue
        xy.append(np.linalg.norm(e.centre[:2] - cube.centre[:2]))
        yaw.append(abs(sim.yaw_error(e.yaw, cube.yaw)))
        faces.append(len(e.faces))
        if verbose and (xy[-1] > 0.004 or yaw[-1] > np.radians(4)):
            print(f"  #{i}: xy {xy[-1] * 1000:.1f} mm, yaw {np.degrees(yaw[-1]):.1f} deg, faces {e.faces}, "
                  f"eye {np.round(t_wc, 3)}")
    return {"xy": np.array(xy), "yaw": np.array(yaw), "faces": np.array(faces), "missed": missed, "wrong": wrong, "n": n}


def run_tracked(n=100, views=6, seed=1, imp=None):
    """Each of n imperfect cubes seen from `views` random wrist poses; the per-frame estimates go
    through a CubeTrack (cube_tracking.py) -> the tracked errors, as run()."""
    import cube_tracking as ct
    imp = sim.Imperfections() if imp is None else imp
    detector = sim.cd.MarkerDetector()
    K, D = sim.camera()
    rng = np.random.default_rng(seed)
    xy, yaw, faces, missed, stable = [], [], [], 0, 0
    for i in range(n):
        cube, _, _ = sim.random_scene(rng, imp)
        track = ct.CubeTrack(ct.TrackerParams(min_obs=3))
        for v in range(views):
            R_wc, t_wc = sim.random_view(rng, cube.centre)
            gray = sim.render([cube], R_wc, t_wc, K, D, rng=rng)
            e = estimate(sim.cd, detector, gray, K, D, R_wc, t_wc, sim.EDGE / 2).get(cube.k)
            if e is not None:
                track.update(ct.Observation(float(v), e.centre, e.yaw, e.confidence, e.face_spread, e.face_spread_yaw))
        if track.centre is None:
            missed += 1
            continue
        xy.append(np.linalg.norm(track.centre[:2] - cube.centre[:2]))
        yaw.append(abs(sim.yaw_error(track.yaw, cube.yaw)))
        faces.append(track.n_obs)
        stable += track.stable
    return {"xy": np.array(xy), "yaw": np.array(yaw), "faces": np.array(faces), "missed": missed, "wrong": 0,
            "n": n, "stable": stable}


def summary(r):
    xy, yaw = r["xy"] * 1000, np.degrees(r["yaw"])
    q = lambda a, p: np.percentile(a, p) if len(a) else float("nan")  # noqa: E731
    return (f"detected {len(xy)}/{r['n']} (wrong cube {r['wrong']})  "
            f"xy mm p50 {q(xy, 50):.2f} p95 {q(xy, 95):.2f} max {xy.max():.2f}  "
            f"yaw deg p50 {q(yaw, 50):.2f} p95 {q(yaw, 95):.2f} max {yaw.max():.2f}  "
            f"<=3mm {np.mean(xy <= 3) * 100:.0f}%  <=3deg {np.mean(yaw <= 3) * 100:.0f}%")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--perfect", action="store_true", help="ideal cubes (no paper imperfections)")
    ap.add_argument("--module", help="cube_detection.py to test instead of the one in scripts/")
    ap.add_argument("--views", type=int, default=0, help="also run the tracked test with this many views per cube")
    ap.add_argument("-v", "--verbose", action="store_true", help="print the bad samples")
    args = ap.parse_args()
    module = load_module(args.module) if args.module else None
    imp = sim.Imperfections.none() if args.perfect else sim.Imperfections()
    print("per frame:", summary(run(args.n, args.seed, imp, module, args.verbose)))
    if args.views:
        r = run_tracked(max(args.n // args.views, 20), args.views, args.seed, imp)
        print(f"tracked, {args.views} views per cube:", summary(r), f" stable {r['stable']}/{len(r['xy'])}")


if __name__ == "__main__":
    main()
