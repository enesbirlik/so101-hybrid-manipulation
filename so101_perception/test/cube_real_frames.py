#!/usr/bin/env python3
"""Cube detector on saved wrist-camera frames, offline (no ROS, no bus).

    python3 so101_perception/test/cube_real_frames.py ~/so101_data/cubes/20261122 [--glob 'rest_*.jpg']
            [--joints rest_joints.json] [--refinement subpix|contour|apriltag|none] [--overlay out_dir]

The folder holds frames as published (upright, any size) and a JSON of the joint angles (rad) at
which they were taken. The camera pose comes from forward kinematics of so101_new_calib.urdf with
the joint origins of so101_kinematics.yaml and the hand-eye transform, as in tools/so101_hand_eye.py.

Per cube it prints the fused centre and yaw (mean and spread over the frames), and how well its
faces agree: the centre and yaw each face implies alone. That spread is the paper cube's own
imperfection (plus the corner noise), independent of the camera pose error.
"""

import argparse
import glob
import json
import os
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "so101_perception", "scripts"))
import cube_detection as cd  # noqa: E402

URDF = os.path.join(REPO, "so101_description", "urdf", "so101_new_calib.urdf")
KINEMATICS = os.path.join(REPO, "so101_description", "config", "so101_kinematics.yaml")
HAND_EYE = os.path.join(REPO, "so101_perception", "config", "wrist_camera_hand_eye.yaml")
INTRINSICS = os.path.join(REPO, "so101_perception", "config", "wrist_camera_fisheye.yaml")
CUBES = os.path.join(REPO, "so101_perception", "config", "cubes.yaml")


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quaternion_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def camera_pose(q):
    """Joint angles (rad) -> (R_wc, t_wc) of wrist_camera_optical_frame in base_link, and table_z."""
    with open(KINEMATICS) as f:
        kin = yaml.safe_load(f)
    joints = [j for j in ET.parse(URDF).getroot().iter("joint") if j.find("child") is not None]
    by_child = {j.find("child").get("link"): j for j in joints}
    chain, link = [], "gripper_link"
    while link != "base_link":
        j = by_child[link]
        axis = None if j.get("type") == "fixed" else np.array([float(v) for v in j.find("axis").get("xyz").split()])
        if j.get("name") in kin["joints"]:
            o = kin["joints"][j.get("name")]
            origin = transform(rpy_matrix(*o["rpy"]), o["xyz"])
        else:
            o = j.find("origin")
            origin = transform(rpy_matrix(*map(float, o.get("rpy").split())), [float(v) for v in o.get("xyz").split()])
        chain.append((j.get("name"), origin, axis))
        link = j.find("parent").get("link")
    T = np.eye(4)
    for name, origin, axis in chain[::-1]:
        T = T @ origin
        if axis is not None:
            T = T @ transform(cv2.Rodrigues(axis * q[name])[0], np.zeros(3))
    with open(HAND_EYE) as f:
        he = yaml.safe_load(f)
    r, t = he["rotation"], he["translation"]
    T = T @ transform(quaternion_matrix(r["x"], r["y"], r["z"], r["w"]), [t["x"], t["y"], t["z"]])
    return T[:3, :3], T[:3, 3], float(kin["table_z"])


def intrinsics(width):
    with open(INTRINSICS) as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
    return cd.scaled_camera(K, width, c["image_width"]), D


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--glob", default="*.jpg")
    ap.add_argument("--joints", default="rest_joints.json", help="JSON of joint angles (rad), in the folder")
    ap.add_argument("--refinement", default="subpix", choices=list(cd.MarkerDetector.REFINEMENTS))
    ap.add_argument("--no-table", action="store_true", help="do not use the table prior")
    ap.add_argument("--overlay", help="write the frames with the detections drawn in to this folder")
    args = ap.parse_args()

    with open(os.path.join(args.folder, args.joints)) as f:
        q = json.load(f)
    R_wc, t_wc, table_z = camera_pose(q)
    geometry = cd.load_cube_geometry(CUBES) if os.path.exists(CUBES) else None
    det = cd.MarkerDetector(refinement=args.refinement)
    print(f"camera in base_link {np.round(t_wc, 4)} m, looking along {np.round(R_wc[:, 2], 2)}; table_z {table_z}")
    per_cube = {}
    paths = sorted(glob.glob(os.path.join(args.folder, args.glob)))
    for path in paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        K, D = intrinsics(gray.shape[1])
        detections = det.detect(gray)
        poses = cd.marker_poses(detections, K, D, geometry)
        cubes = cd.cube_estimates(poses, R_wc, t_wc, table_z=None if args.no_table else table_z)
        for e in cubes:
            per_cube.setdefault(e.cube, []).append(e)
        if args.overlay:
            os.makedirs(args.overlay, exist_ok=True)
            for marker_id, px in detections:
                cv2.polylines(img, [px.astype(np.int32)], True, (0, 200, 0), 1)
                cv2.putText(img, str(marker_id), tuple(px.mean(axis=0).astype(int) - [6, -4]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(args.overlay, os.path.basename(path)), img)
    print(f"{len(paths)} frames\n")
    print("cube  frames  centre x, y [mm]   sd    yaw [deg]  sd   | faces (rejected)  top  rms px  conf"
          "  | face-implied centres: spread [mm], yaw spread [deg]")
    spreads = []
    for k, es in sorted(per_cube.items()):
        C = np.array([e.centre[:2] for e in es]) * 1000
        Y = np.degrees([e.yaw for e in es])
        ym, _ = cd.circular_mean_90(np.radians(Y))
        ysd = np.degrees(np.sqrt(np.mean([cd.fold_yaw(np.radians(y) - ym) ** 2 for y in Y])))
        faces = sorted({i for e in es for i in e.faces})
        rejected = sorted({i for e in es for i in e.rejected})
        sp = np.mean([e.face_spread for e in es]) * 1000
        spy = np.degrees(np.mean([e.face_spread_yaw for e in es]))
        if len(es[0].faces) > 1:
            spreads.append(sp)
        e = es[0]
        print(f"K{k}   {len(es):4d}    {C[:, 0].mean():6.1f} {C[:, 1].mean():6.1f}   {np.linalg.norm(C.std(0)):4.2f}"
              f"   {np.degrees(ym):+6.1f}  {ysd:4.2f}  | {faces} ({rejected or '-'})  {e.top_face:3d}  "
              f"{np.mean([x.reproj_px for x in es]):5.2f}  {np.mean([x.confidence for x in es]):4.2f}  | {sp:4.1f}, {spy:4.1f}")
        for i, c in e.face_centres.items():
            print(f"        face {i:2d}: centre {c[0] * 1000:6.1f} {c[1] * 1000:6.1f}  yaw {np.degrees(e.face_yaws[i]):+6.1f}")
    if spreads:
        print(f"\nface spread of the cubes seen with 2+ faces: mean {np.mean(spreads):.1f} mm, max {np.max(spreads):.1f} mm")


if __name__ == "__main__":
    main()
