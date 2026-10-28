#!/usr/bin/env python3
"""Hand-eye calibration of the wrist camera: the camera pose relative to gripper_link.

  capture  the ChArUco board lies fixed on the table; move the arm by hand so
           the camera sees it from many angles; SPACE saves the joint ticks and
           the board corners once the arm is still
  auto     torque on: set a centre pose looking at the board, then the arm
           visits random poses around it on its own, reaching each one from
           both sides, and captures them (needs a rough result from
           capture + solve to keep the board in view)
  solve    cv2.calibrateHandEye (eye-in-hand) on the saved samples -> YAML;
           for auto samples it also reports the play between both approaches

The board must not move during a capture session: the solver relies on it
staying at one place in base_link. Vary the gripper orientation as much as
possible (tilt, roll, different sides of the board); translation alone
cannot pin the camera rotation down.

Prefer auto. With torque off, gravity and the hand push each joint around
inside its gear play and wobble the encoder does not see; with torque on the
servo holds its output shaft, and nobody touches the arm.

Forward kinematics come straight from so101_new_calib.urdf (base_link ->
gripper_link) and the joint calibration file, so ROS does not need to run.
Samples keep raw ticks and corners, so `solve` can be rerun after a joint or
camera recalibration without capturing again.

The result is the OpenCV optical frame of the upright (rotated) image:
x right, y down, z forward.

Opens the serial bus directly: never run it while so101_hardware is running.
"""

import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

import feetech_bus as fb
from feetech_bus import FeetechBus, FeetechError
import so101_camera_calib as cam
from so101_calibrate import DEFAULT_FILE as JOINT_FILE, JOINTS, TICKS_PER_DEG, Abort, hold_all, load_calibration, \
    move_joints, to_deg

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "so101_description" / "urdf" / "so101_new_calib.urdf"
CAMERA_XACRO = REPO / "so101_description" / "urdf" / "wrist_camera.xacro"
DEFAULT_SAMPLES = Path.home() / "so101_data" / "hand_eye"
DEFAULT_FILE = REPO / "so101_perception" / "config" / "wrist_camera_hand_eye.yaml"

BASE, TOOL = "base_link", "gripper_link"
MIN_CORNERS = 10         # of 24; a board pose from fewer is too noisy
MIN_SAMPLES = 8
STILL_WINDOW = 0.4       # s; the arm counts as still when no joint moved more
STILL_TICKS = 2          # than this (~0.2 deg) within the window
OUTLIER_FACTOR = 3.0     # samples whose board position is off by more than this
OUTLIER_FLOOR_MM = 5.0   # times the median (and this much) are dropped once

# auto: random poses around the centre pose, at most this far per joint [deg]
AUTO_SPREAD_DEG = {"shoulder_pan": 15, "shoulder_lift": 10, "elbow_flex": 10, "wrist_flex": 25, "wrist_roll": 45}
APPROACH_DEG = 3.0       # each pose is reached once from above and once from below
RANGE_MARGIN_DEG = 5.0   # stay this far inside the calibrated range
MIN_HEIGHT = 0.05        # m; gripper_link and gripper_frame_link stay this high above base_link
SETTLE_S = 1.0           # wait after arriving, before the still check and the frame
MIN_VISIBLE = 0.7        # predicted share of board corners inside the image
# Predicted by the unverified encoder-wrap theory (devlog day 1): with torque on these
# joints jerked into their stops outside (lower, upper) [deg]. Auto keeps away from there.
# Angles of the current calibration: the shoulder_lift zero moved +10.5 deg on 2026-10-28,
# so the old -98 deg is now -87.5 deg (same encoder position).
JERK_SAFE_DEG = {"shoulder_pan": (-8, None), "shoulder_lift": (-87.5, None), "elbow_flex": (None, 83),
                 "wrist_flex": (-13, None), "wrist_roll": (None, 25)}
JERK_MARGIN_DEG = 3.0
# solve --refine-joints: zero offsets that can be told apart from the camera and board
# poses. Pan and roll offsets cannot: they just turn the board (about the vertical) or the camera.
REFINE_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
# The board lies on the table the arm stands on. base_link is 2.4 mm above the lowest point
# of the base meshes, about the thickness of the board on its backing, so its top is at ~0.
BOARD_Z = 0.0

METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# --- kinematics --------------------------------------------------------------

def rpy_matrix(r, p, y):
    """URDF convention: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_rpy(R):
    return [float(np.arctan2(R[2, 1], R[2, 2])), float(np.arcsin(-R[2, 0])), float(np.arctan2(R[1, 0], R[0, 0]))]


def matrix_quaternion(R):
    """(x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [float(x), float(y), float(z), float(w)]


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.ravel(t)
    return T


def origin_transform(joint):
    o = joint.find("origin")
    if o is None:
        return np.eye(4)
    return transform(rpy_matrix(*map(float, o.get("rpy", "0 0 0").split())),
                     [float(v) for v in o.get("xyz", "0 0 0").split()])


def load_chain(path, base, tool):
    """Joints from base to tool as (name, fixed origin, axis or None for fixed joints)."""
    # iter() also reaches into xacro macros; <joint> inside <transmission> has no child
    joints = [j for j in ET.parse(path).getroot().iter("joint") if j.find("child") is not None]
    by_child = {j.find("child").get("link"): j for j in joints}
    chain, link = [], tool
    while link != base:
        j = by_child[link]
        axis = None if j.get("type") == "fixed" else np.array([float(v) for v in j.find("axis").get("xyz").split()])
        chain.append((j.get("name"), origin_transform(j), axis))
        link = j.find("parent").get("link")
    return chain[::-1]


def forward(chain, q):
    T = np.eye(4)
    for name, origin, axis in chain:
        T = T @ origin
        if axis is not None:
            T = T @ transform(cv2.Rodrigues(axis * q[name])[0])
    return T


def urdf_camera():
    """gripper_link -> wrist_camera_link from the xacro (a mesh frame, not the optical frame)."""
    return forward(load_chain(CAMERA_XACRO, "${parent}", "wrist_camera_link"), {})


def to_rad(ticks, cal):
    return {n: cal[n]["direction"] * (t - cal[n]["zero"]) * 2 * np.pi / fb.TICKS_PER_TURN for n, t in ticks.items()}


def to_ticks(q_deg, cal):
    return {n: round(cal[n]["zero"] + cal[n]["direction"] * v * TICKS_PER_DEG) for n, v in q_deg.items()}


def rotation_angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def mean_rotation(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    return U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt


# --- camera ------------------------------------------------------------------

def load_intrinsics(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    if (data["image_width"], data["image_height"]) != (cam.WIDTH, cam.HEIGHT):
        sys.exit(f"{path} is for {data['image_width']}x{data['image_height']}, the camera runs {cam.WIDTH}x{cam.HEIGHT}")
    return np.array(data["camera_matrix"]["data"]).reshape(3, 3), np.array(data["distortion_coefficients"]["data"]).reshape(4, 1)


def load_hand_eye(path):
    if not path.exists():
        return None
    with open(path) as f:
        data = yaml.safe_load(f)
    r, t = data["rotation"], data["translation"]
    return transform(quaternion_matrix(r["x"], r["y"], r["z"], r["w"]), [t["x"], t["y"], t["z"]])


def board_visible(T_cb, board_points, K, D, margin=30):
    """Share of board corners a camera at T_cb (camera -> board) would see."""
    pts = board_points @ T_cb[:3, :3].T + T_cb[:3, 3]
    if (pts[:, 2] < 0.05).any():
        return 0.0
    p, _ = cv2.fisheye.projectPoints(board_points.reshape(-1, 1, 3), cv2.Rodrigues(T_cb[:3, :3])[0],
                                     np.ascontiguousarray(T_cb[:3, 3]).reshape(3, 1), K, D)
    p = p.reshape(-1, 2)
    inside = (p[:, 0] > margin) & (p[:, 0] < cam.WIDTH - margin) & (p[:, 1] > margin) & (p[:, 1] < cam.HEIGHT - margin)
    return float(inside.mean())


def board_pose(corners, ids, board_points, K, D):
    """camera -> board transform from the fisheye-undistorted corners, and its reprojection RMS [px]."""
    obj = board_points[ids.flatten()]
    img = corners.reshape(-1, 1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, cv2.fisheye.undistortPoints(img, K, D), np.eye(3), None)
    proj, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D)
    rms = float(np.sqrt(((proj - img) ** 2).sum(-1).mean()))
    return transform(cv2.Rodrigues(rvec)[0], tvec), rms


def draw_axes(image, T, K, D, length=0.05):
    pts = np.array([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]], np.float64).reshape(-1, 1, 3)
    rvec = cv2.Rodrigues(T[:3, :3])[0]
    tvec = np.ascontiguousarray(T[:3, 3]).reshape(3, 1)   # fisheye.projectPoints rejects a strided column
    p, _ = cv2.fisheye.projectPoints(pts, rvec, tvec, K, D)
    p = [tuple(map(int, q)) for q in p.reshape(-1, 2)]
    for end, color in zip(p[1:], [(0, 0, 255), (0, 255, 0), (255, 0, 0)]):
        cv2.line(image, p[0], end, color, 3)


# --- commands ----------------------------------------------------------------

def setup(args):
    cal = load_calibration(args.joint_file)
    chain = load_chain(URDF, BASE, TOOL)
    names = [n for n, _, axis in chain if axis is not None]
    missing = [n for n in names if n not in cal]
    if missing:
        sys.exit(f"not calibrated in {args.joint_file}: {', '.join(missing)}")
    K, D = load_intrinsics(args.intrinsics)
    board, dictionary = cam.make_board(args.square_mm)
    return cal, chain, names, K, D, board, dictionary, np.asarray(board.chessboardCorners, np.float64)


def load_samples(folder):
    index = folder / "samples.json"
    return json.loads(index.read_text()) if index.exists() else []


def cmd_capture(args):
    cal, chain, names, K, D, board, dictionary, board_points = setup(args)
    args.samples.mkdir(parents=True, exist_ok=True)
    index = args.samples / "samples.json"
    samples = load_samples(args.samples)
    saved_R = [forward(chain, to_rad(s["ticks"], cal))[:3, :3] for s in samples]
    history = deque()
    with FeetechBus(args.port or fb.find_port()) as bus:
        if args.torque_off:
            for n in names:
                bus.write_u8(cal[n]["id"], fb.TORQUE_ENABLE, 0)
        cap = cam.open_camera(args.device)
        cv2.namedWindow("hand-eye", cv2.WINDOW_NORMAL)
        print(f"saving to {args.samples} ({len(samples)} samples there already)")
        print("SPACE save (arm still, board seen), d delete last, q quit. Keep the board fixed on the table.")
        while True:
            ok, frame = cam.read(cap)
            if not ok:
                continue
            try:
                ticks = {n: bus.read_position(cal[n]["id"]) for n in names}
            except FeetechError:
                continue
            now = time.monotonic()
            history.append((now, ticks))
            while now - history[0][0] > STILL_WINDOW:
                history.popleft()
            still = now - history[0][0] > 0.8 * STILL_WINDOW and all(
                max(h[n] for _, h in history) - min(h[n] for _, h in history) <= STILL_TICKS for n in names)
            R = forward(chain, to_rad(ticks, cal))[:3, :3]
            novelty = min((rotation_angle(R.T @ S) for S in saved_R), default=180.0)

            corners, ids = cam.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), board, dictionary)
            n = 0 if ids is None else len(ids)
            view = frame.copy()
            rms = None
            if n >= MIN_CORNERS:
                T_cam_board, rms = board_pose(corners, ids, board_points, K, D)
                cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
                draw_axes(view, T_cam_board, K, D)
            ready = n >= MIN_CORNERS and still
            lines = [
                (f"corners {n}/24" + (f"  pose rms {rms:.2f} px" if rms is not None else ""), n >= MIN_CORNERS),
                ("still" if still else "moving", still),
                (f"{novelty:.0f} deg from nearest sample (aim > 15)", novelty > 15),
                (f"samples {len(samples)}", len(samples) >= MIN_SAMPLES),
            ]
            for k, (text, good) in enumerate(lines):
                cv2.putText(view, text, (20, 40 + 35 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 255, 0) if good else (0, 0, 255), 2)
            cv2.imshow("hand-eye", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("d") and samples:
                gone = samples.pop()
                saved_R.pop()
                (args.samples / gone["image"]).unlink(missing_ok=True)
                index.write_text(json.dumps(samples, indent=1))
                print(f"  deleted {gone['image']}, {len(samples)} left")
            if key == ord(" "):
                if not ready:
                    print(f"  not saved: {'arm moving' if not still else f'{n} corners, need {MIN_CORNERS}'}")
                    continue
                name = f"sample_{datetime.now():%Y%m%d_%H%M%S_%f}.png"
                cv2.imwrite(str(args.samples / name), frame)
                samples.append({"image": name, "ticks": ticks})
                saved_R.append(R)
                index.write_text(json.dumps(samples, indent=1))
                print(f"  {len(samples):3d}  {name}  {n} corners  pose rms {rms:.2f} px  {novelty:.0f} deg new")
        cap.release()
        cv2.destroyAllWindows()


def pose_problem(q_deg, cal, chains):
    """Why the arm must not go to q_deg [deg], or None."""
    for n, v in q_deg.items():
        lo, hi = sorted((to_deg(cal[n]["range_min"], cal[n]), to_deg(cal[n]["range_max"], cal[n])))
        if not lo + RANGE_MARGIN_DEG <= v <= hi - RANGE_MARGIN_DEG:
            return f"{n} {v:+.0f} deg is outside its range"
        safe_lo, safe_hi = JERK_SAFE_DEG.get(n, (None, None))
        if (safe_lo is not None and v < safe_lo + JERK_MARGIN_DEG) or (safe_hi is not None and v > safe_hi - JERK_MARGIN_DEG):
            return f"{n} {v:+.0f} deg is in its predicted jerk zone"
    q = {n: np.radians(v) for n, v in q_deg.items()}
    if min(forward(c, q)[2, 3] for c in chains) < MIN_HEIGHT:
        return "too close to the table"
    return None


def plan_poses(centre, count, rng, allowed):
    """Random poses around centre that pass allowed() together with both approach poses, in a short visiting order."""
    poses = []
    for _ in range(5000):
        if len(poses) == count:
            break
        q = {n: v + rng.uniform(-AUTO_SPREAD_DEG[n], AUTO_SPREAD_DEG[n]) for n, v in centre.items()}
        if all(allowed({n: v + s * APPROACH_DEG for n, v in q.items()}) for s in (0, 1, -1)):
            poses.append(q)
    ordered, current = [], centre
    while poses:
        k = min(range(len(poses)), key=lambda i: max(abs(poses[i][n] - current[n]) for n in current))
        current = poses.pop(k)
        ordered.append(current)
    return ordered


def teach_centre(bus, cap, cal, names, chains, board, dictionary, board_points, K, D):
    """Live view until SPACE on an allowed pose that sees the board: (pose [deg], camera -> board), or None."""
    print("Move the arm so the camera looks at the board from 20-30 cm, then SPACE in the window. q quits.")
    while True:
        ok, frame = cam.read(cap)
        if not ok:
            continue
        try:
            q = {n: to_deg(bus.read_position(cal[n]["id"]), cal[n]) for n in names}
        except FeetechError:
            continue
        corners, ids = cam.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), board, dictionary)
        n = 0 if ids is None else len(ids)
        problem = pose_problem(q, cal, chains)
        view = frame.copy()
        T_cb = None
        if n >= MIN_CORNERS:
            T_cb, _ = board_pose(corners, ids, board_points, K, D)
            cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
            draw_axes(view, T_cb, K, D)
        for k, (text, good) in enumerate([(f"corners {n}/24", n >= MIN_CORNERS), (problem or "pose ok", not problem)]):
            cv2.putText(view, text, (20, 40 + 35 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if good else (0, 0, 255), 2)
        cv2.imshow("hand-eye", view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return None
        if key == ord(" "):
            if problem or T_cb is None:
                print(f"  not this pose: {problem or 'board not seen'}")
                continue
            return q, T_cb


def grab_still(bus, cap, cal, names):
    """Frame and joint ticks after settling; the frame is None if a joint still moved."""
    time.sleep(SETTLE_S)
    before = {n: bus.read_position(cal[n]["id"]) for n in names}
    for _ in range(5):
        cap.grab()  # drop frames queued during the move
    ok, frame = cam.read(cap)
    after = {n: bus.read_position(cal[n]["id"]) for n in names}
    if not ok or any(abs(after[n] - before[n]) > STILL_TICKS for n in names):
        return None, after
    return frame, after


def cmd_auto(args):
    cal, chain, names, K, D, board, dictionary, board_points = setup(args)
    T_gc = load_hand_eye(args.file)
    if T_gc is None:
        sys.exit(f"no rough result in {args.file}: run capture + solve once first, auto uses it to keep the board in view")
    chains = (chain, load_chain(URDF, BASE, "gripper_frame_link"))
    joints = [j for j in JOINTS if j.name in names]
    args.samples.mkdir(parents=True, exist_ok=True)
    index = args.samples / "samples.json"
    samples = load_samples(args.samples)
    run = f"{datetime.now():%Y%m%d_%H%M%S}"
    with FeetechBus(args.port or fb.find_port()) as bus:
        if args.torque_off:
            for j in joints:
                bus.write_u8(j.id, fb.TORQUE_ENABLE, 0)
        cap = cam.open_camera(args.device)
        cv2.namedWindow("hand-eye", cv2.WINDOW_NORMAL)
        started = False   # torque switched on, the arm moves
        try:
            taught = teach_centre(bus, cap, cal, names, chains, board, dictionary, board_points, K, D)
            if taught is None:
                return
            centre, T_cb0 = taught
            T_bt = forward(chain, {n: np.radians(v) for n, v in centre.items()}) @ T_gc @ T_cb0

            def allowed(q):
                if pose_problem(q, cal, chains):
                    return False
                T_bg = forward(chain, {n: np.radians(v) for n, v in q.items()})
                return board_visible(np.linalg.inv(T_bg @ T_gc) @ T_bt, board_points, K, D) >= MIN_VISIBLE

            poses = plan_poses(centre, args.poses, np.random.default_rng(args.seed), allowed)
            if not poses:
                sys.exit("no allowed pose around this centre: pick one further from the joint limits and jerk zones")
            print(f"{len(poses)} poses, each reached from both sides: {2 * len(poses)} samples, "
                  f"about {len(poses) * 12 // 60 + 1} min.")
            print("The arm moves on its own. Stay next to it: q in the window or Ctrl+C freezes it in place.")
            input("Enter switches torque on and starts ")
            hold_all(bus, joints)
            started = True
            for k, q in enumerate(poses, 1):
                for sign, label in ((1, "+"), (-1, "-")):
                    for target in ({n: v + sign * APPROACH_DEG for n, v in q.items()}, q):
                        stuck = move_joints(bus, cal, joints, to_ticks(target, cal))
                        if stuck:
                            raise Abort(f"stuck: {', '.join(stuck)}")
                    frame, ticks = grab_still(bus, cap, cal, names)
                    if frame is None:
                        print(f"  {k:2d}{label}  skipped: the arm was still moving")
                        continue
                    corners, ids = cam.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), board, dictionary)
                    n = 0 if ids is None else len(ids)
                    view = frame.copy()
                    if n >= MIN_CORNERS:
                        name = f"auto_{run}_{k:02d}{'p' if sign > 0 else 'm'}.png"
                        cv2.imwrite(str(args.samples / name), frame)
                        samples.append({"image": name, "ticks": ticks, "pose": f"{run}-{k}", "approach": label})
                        index.write_text(json.dumps(samples, indent=1))
                        cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
                    print(f"  {k:2d}{label}  {n} corners" + ("" if n >= MIN_CORNERS else ", skipped"))
                    cv2.putText(view, f"pose {k}/{len(poses)} {label}   corners {n}   samples {len(samples)}",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    cv2.imshow("hand-eye", view)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        raise Abort("stopped")
            print("done, back to the centre pose")
            move_joints(bus, cal, joints, to_ticks(centre, cal))
        except (Abort, KeyboardInterrupt) as e:
            if started:
                for j in joints:
                    bus.write_u16(j.id, fb.GOAL_POSITION, bus.read_position(j.id))
            print(f"\n{str(e) or 'interrupted'}" + (": frozen in place, holding." if started else ", the arm did not move."))
        finally:
            cap.release()
            cv2.destroyAllWindows()
    print(f"{len(samples)} samples in {args.samples}.")
    if started:
        print("Torque stays on. Release: python3 tools/so101_calibrate.py off (support the arm first)")


def play_report(used, T_bg, T_cb):
    """Auto samples reach each pose from both sides. The joints may stop a few ticks apart
    (measured, harmless); what the camera sees beyond that is play the encoders miss."""
    sides = {}
    for s, a, b in zip(used, T_bg, T_cb):
        if "pose" in s:
            sides.setdefault(s["pose"], {})[s["approach"]] = (a, b)
    pairs = [p for p in sides.values() if len(p) == 2]
    if not pairs:
        return
    arm = np.array([rotation_angle(p["+"][0][:3, :3].T @ p["-"][0][:3, :3]) for p in pairs])
    seen = np.array([rotation_angle(p["+"][1][:3, :3] @ p["-"][1][:3, :3].T) for p in pairs])
    eye = [[-T[:3, :3].T @ T[:3, 3] for T in (p["+"][1], p["-"][1])] for p in pairs]   # camera in board frame
    shift = np.array([np.linalg.norm(a - b) * 1000 for a, b in eye])
    unexplained = np.abs(seen - arm)
    print(f"\nplay, same pose from both sides ({len(pairs)} poses), median / max:")
    print(f"  joints (encoders)  {np.median(arm):5.2f} / {arm.max():5.2f} deg")
    print(f"  camera (board)     {np.median(seen):5.2f} / {seen.max():5.2f} deg, {np.median(shift):4.1f} / {shift.max():4.1f} mm")
    print(f"  missed by encoders {np.median(unexplained):5.2f} / {unexplained.max():5.2f} deg")


def board_in_base(T_bg, T_gc, T_cb):
    """Where each sample puts the board in base_link; position spread [mm], rotation spread [deg], per-sample mm."""
    T = [a @ T_gc @ b for a, b in zip(T_bg, T_cb)]
    p = np.array([t[:3, 3] for t in T])
    dist = np.linalg.norm(p - p.mean(0), axis=1) * 1000
    Rm = mean_rotation([t[:3, :3] for t in T])
    ang = np.array([rotation_angle(Rm.T @ t[:3, :3]) for t in T])
    return float(np.sqrt((dist ** 2).mean())), float(np.sqrt((ang ** 2).mean())), dist


def solve_all(T_bg, T_cb):
    results = {}
    for name, method in METHODS.items():
        R, t = cv2.calibrateHandEye([T[:3, :3] for T in T_bg], [T[:3, 3] for T in T_bg],
                                    [T[:3, :3] for T in T_cb], [T[:3, 3] for T in T_cb], method=method)
        T_gc = transform(R, t)
        results[name] = (T_gc, *board_in_base(T_bg, T_gc, T_cb))
    return results


def pose_vector(T):
    return np.r_[cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]]


def pose_matrix(v):
    return transform(cv2.Rodrigues(np.asarray(v[:3], float))[0], v[3:6])


def board_on_table(b):
    """Board lying face up on the table at x, y [m] turned by yaw [rad]; its z axis points down."""
    return transform(rpy_matrix(0, 0, b[2]) @ np.diag([1.0, -1.0, -1.0]), [b[0], b[1], BOARD_Z])


def reprojection(chain, cal, samples, obs, K, D, free):
    """Residual function [px] of x = camera pose (6), board x, y, yaw on the table (3), zero offsets of `free` [rad]."""
    q = [to_rad(s["ticks"], cal) for s in samples]

    def residuals(x):
        T_gc, T_bt = pose_matrix(x[:6]), board_on_table(x[6:9])
        offset = dict(zip(free, x[9:]))
        out = []
        for qi, (obj, img) in zip(q, obs):
            T_cb = np.linalg.inv(forward(chain, {n: v + offset.get(n, 0.0) for n, v in qi.items()}) @ T_gc) @ T_bt
            p, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), cv2.Rodrigues(T_cb[:3, :3])[0],
                                             np.ascontiguousarray(T_cb[:3, 3]).reshape(3, 1), K, D)
            out.append((p.reshape(-1, 2) - img).ravel())
        return np.concatenate(out)
    return residuals


def rms_px(residuals):
    return float(np.sqrt((residuals.reshape(-1, 2) ** 2).sum(1).mean()))


def fit_offsets(chain, cal, samples, obs, x0, K, D, free):
    """(solution, standard deviation of each parameter) of the reprojection fit."""
    fit = least_squares(reprojection(chain, cal, samples, obs, K, D, free), x0[:9 + len(free)], loss="huber", f_scale=3.0)
    dof = max(1, len(fit.fun) - len(fit.x))
    cov = np.linalg.pinv(fit.jac.T @ fit.jac) * (fit.fun ** 2).sum() / dof
    return fit, np.sqrt(np.diag(cov))


def refine_report(chain, cal, used, obs, T_gc, T_bg, T_cb, K, D):
    """Fit camera pose, board pose and joint zero offsets together on the corner reprojection.

    The board is held on the table plane (z = BOARD_Z, horizontal). With a free board pose
    a shoulder_lift offset just tilts and lifts the board and comes out different in every
    run; on the table it is pinned down (devlog day 3). Report only: apply an offset to the
    joint calibration by hand, and only when all runs agree on it, then solve again."""
    T_bt = [a @ T_gc @ b for a, b in zip(T_bg, T_cb)]
    p = np.mean([T[:3, 3] for T in T_bt], 0)
    R = mean_rotation([T[:3, :3] for T in T_bt])
    x0 = np.r_[pose_vector(T_gc), p[0], p[1], np.arctan2(R[1, 0], R[0, 0]), np.zeros(len(REFINE_JOINTS))]
    print("\njoint zero refinement (corner reprojection, board on the table, all samples):")
    for free in ([], REFINE_JOINTS):
        fit, sd = fit_offsets(chain, cal, used, obs, x0, K, D, free)
        print(f"  offsets {'free' if free else 'zero'}: {rms_px(fit.fun):.2f} px")
    offsets = dict(zip(REFINE_JOINTS, fit.x[9:]))
    for n, sd_n in zip(REFINE_JOINTS, sd[9:]):
        c = cal[n]
        new_zero = round(c["zero"] - c["direction"] * np.degrees(offsets[n]) * TICKS_PER_DEG)
        print(f"    {n:<14}{np.degrees(offsets[n]):+6.2f} deg (+-{np.degrees(sd_n):.2f})   zero {c['zero']} -> {new_zero}")
    p = pose_matrix(fit.x[:6])[:3, 3] * 1000
    print(f"    camera at [{p[0]:6.1f} {p[1]:6.1f} {p[2]:6.1f}] mm with these offsets")

    # Held out: fit on every other pose, test on the rest. Both sides of an auto pose stay together.
    groups = sorted({s.get("pose", s["image"]) for s in used})
    train = [k for k, s in enumerate(used) if groups.index(s.get("pose", s["image"])) % 2 == 0]
    test = [k for k in range(len(used)) if k not in train]
    sub = lambda ks: ([used[k] for k in ks], [obs[k] for k in ks])
    line = []
    for free in ([], REFINE_JOINTS):
        fit, _ = fit_offsets(chain, cal, *sub(train), x0, K, D, free)
        line.append(rms_px(reprojection(chain, cal, *sub(test), K, D, free)(fit.x)))
    print(f"  held-out poses: {line[0]:.2f} px without offsets, {line[1]:.2f} px with them")

    # Offsets that are real come out the same from every run (centre pose).
    runs = sorted({s.get("pose", "manual").split("-")[0] for s in used})
    if len(runs) > 1:
        print("  per run (should agree):")
        for run in runs:
            ks = [k for k, s in enumerate(used) if s.get("pose", "manual").split("-")[0] == run]
            if len(ks) < MIN_SAMPLES:
                continue
            fit, sd = fit_offsets(chain, cal, *sub(ks), x0, K, D, REFINE_JOINTS)
            print(f"    {run:<16}" + "  ".join(f"{n} {np.degrees(v):+5.1f} (+-{np.degrees(e):.1f})"
                                             for n, v, e in zip(REFINE_JOINTS, fit.x[9:], sd[9:])))


def cmd_solve(args):
    cal, chain, names, K, D, board, dictionary, board_points = setup(args)
    used, T_bg, T_cb, obs = [], [], [], []
    for s in load_samples(args.samples):
        gray = cv2.imread(str(args.samples / s["image"]), cv2.IMREAD_GRAYSCALE)
        corners, ids = cam.detect(gray, board, dictionary) if gray is not None else (None, None)
        if ids is None or len(ids) < MIN_CORNERS:
            print(f"  skip {s['image']}: board not found")
            continue
        used.append(s)
        T_bg.append(forward(chain, to_rad(s["ticks"], cal)))
        T_cb.append(board_pose(corners, ids, board_points, K, D)[0])
        obs.append((board_points[ids.flatten()], corners.reshape(-1, 2).astype(np.float64)))
    print(f"{len(used)} usable samples in {args.samples}")
    if len(used) < MIN_SAMPLES:
        sys.exit(f"need at least {MIN_SAMPLES}, better 15-25")
    spread = max(rotation_angle(a[:3, :3].T @ b[:3, :3]) for a in T_bg for b in T_bg)
    print(f"largest rotation between two gripper poses: {spread:.0f} deg" + ("  (too little, tilt more)" if spread < 45 else ""))

    while True:
        results = solve_all(T_bg, T_cb)
        best = min(results, key=lambda m: results[m][1])
        dist = results[best][3]
        limit = max(OUTLIER_FACTOR * np.median(dist), OUTLIER_FLOOR_MM)
        bad = np.flatnonzero(dist > limit)
        if not len(bad) or len(used) - len(bad) < MIN_SAMPLES:
            break
        for k in sorted(bad, reverse=True):
            print(f"  drop {used[k]['image']}: board {dist[k]:.1f} mm off (limit {limit:.1f})")
            del used[k], T_bg[k], T_cb[k], obs[k]

    print("\nboard position/rotation spread in base_link (lower is better; the board did not move):")
    for name, (T_gc, pos, rot, _) in results.items():
        p = T_gc[:3, 3] * 1000
        print(f"  {name:<11}{pos:6.1f} mm {rot:5.2f} deg   camera at [{p[0]:6.1f} {p[1]:6.1f} {p[2]:6.1f}] mm"
              + ("   <- best" if name == best else ""))
    T_gc, pos, rot, dist = results[best]
    print("\nper sample board offset [mm]:")
    for s, d in zip(used, dist):
        print(f"  {d:6.1f}  {s['image']}")
    play_report(used, T_bg, T_cb)
    u = urdf_camera()[:3, 3] * 1000
    p = T_gc[:3, 3] * 1000
    print(f"\nURDF wrist_camera_link origin [{u[0]:6.1f} {u[1]:6.1f} {u[2]:6.1f}] mm "
          f"(mesh origin, not the optical centre): {np.linalg.norm(p - u):.1f} mm from the result")
    print(f"optical axis (z) in gripper_link: {np.array2string(T_gc[:3, 2], precision=3)}")
    # The board lies on the table the arm stands on, and base_link sits at table level, so
    # both should be ~0. A clear offset or tilt points at a joint zero error (devlog day 3).
    T_bt = [a @ T_gc @ b for a, b in zip(T_bg, T_cb)]
    height = np.mean([T[2, 3] for T in T_bt]) * 1000
    tilt = np.degrees(np.arccos(abs(mean_rotation([T[:3, :3] for T in T_bt])[2, 2])))
    print(f"board in base_link: {height:+.1f} mm above base_link, tilted {tilt:.1f} deg (on the table: ~0, ~0)")

    data = {
        "parent_frame": TOOL,
        "child_frame": "wrist_camera_optical_frame",
        "translation": dict(zip("xyz", [float(v) for v in T_gc[:3, 3]])),
        "rotation": dict(zip("xyzw", matrix_quaternion(T_gc[:3, :3]))),
        "rpy": matrix_rpy(T_gc[:3, :3]),
    }
    args.file.parent.mkdir(parents=True, exist_ok=True)
    with open(args.file, "w") as f:
        f.write(f"# so101_hand_eye.py, {datetime.now():%Y-%m-%d %H:%M}, method {best}, {len(used)} samples\n")
        f.write(f"# board spread in base_link: {pos:.1f} mm, {rot:.2f} deg (RMS)\n")
        f.write("# child frame = OpenCV optical frame of the upright 1280x720 image (x right, y down, z forward)\n")
        f.write("# rpy is URDF order (R = Rz(yaw) Ry(pitch) Rx(roll)), metres and radians\n")
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    print(f"saved {args.file}")
    if args.refine_joints:
        refine_report(chain, cal, used, obs, T_gc, T_bg, T_cb, K, D)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--device", default=cam.DEFAULT_DEVICE, help="V4L2 device (default: the wrist camera)")
    parser.add_argument("--samples", type=Path, default=DEFAULT_SAMPLES, help=f"sample folder (default: {DEFAULT_SAMPLES})")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"result YAML (default: {DEFAULT_FILE})")
    parser.add_argument("--joint-file", type=Path, default=JOINT_FILE, help="joint calibration file")
    parser.add_argument("--intrinsics", type=Path, default=cam.DEFAULT_FILE, help="camera intrinsics YAML")
    parser.add_argument("--square-mm", type=float, default=24.0, help="measured printed square side (default: 24)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    capture = sub.add_parser("capture")
    capture.add_argument("--torque-off", action="store_true", help="switch torque off first (support the arm!)")
    auto = sub.add_parser("auto")
    auto.add_argument("--torque-off", action="store_true", help="switch torque off first to set the centre pose by hand (support the arm!)")
    auto.add_argument("--poses", type=int, default=20, help="poses to visit, each from both sides (default: 20)")
    auto.add_argument("--seed", type=int, default=0, help="random seed of the pose plan")
    solve = sub.add_parser("solve")
    solve.add_argument("--refine-joints", action="store_true",
                       help="also estimate zero offsets of " + ", ".join(REFINE_JOINTS) + " (report only)")
    args = parser.parse_args()
    {"capture": cmd_capture, "auto": cmd_auto, "solve": cmd_solve}[args.cmd](args)


if __name__ == "__main__":
    main()
