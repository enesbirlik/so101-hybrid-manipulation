#!/usr/bin/env python3
"""Wrist camera fisheye calibration with the printed ChArUco board.

  capture    live view with the detected corners; SPACE saves the frame
  calibrate  cv2.fisheye calibration on the saved frames -> camera_info YAML
  view       live raw | undistorted view to check the result

Board: calibrx.io 7x5 ChArUco, DICT_6X6_250, same layout as OpenCV's own
CharucoBoard. The PDF says 25 mm squares / 18 mm markers, but printers scale:
measure a printed square and pass it with --square-mm (the marker scales along).

The 135 deg lens needs the fisheye (equidistant) model, so the result has
4 distortion coefficients k1..k4, not the 5 of plumb_bob.

Written for OpenCV 4.6 (legacy cv2.aruco API, no CharucoDetector).
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
DEFAULT_DEVICE = "/dev/v4l/by-id/usb-USB2.0_Camera_USB2.0_Camera_01.00.00-video-index0"
# The 16:9 modes show the whole sensor, centred. Every 4:3 mode is the same crop
# of it, x 78..2958 of 3840, so their principal point sits ~175 px off centre.
# All modes run at 30 fps at most.
WIDTH, HEIGHT = 1280, 720
# The camera is mounted upside down. Frames are turned upright as they are read,
# so the saved frames and the YAML are for the upright image; the ROS driver
# must rotate the same way. Turning by 180 deg keeps fx, fy and k1..k4 and maps
# cx -> W-1-cx, cy -> H-1-cy.
ROTATE_180 = True
DEFAULT_IMAGES = Path.home() / "so101_data" / f"wrist_calib_{WIDTH}x{HEIGHT}"
DEFAULT_FILE = REPO / "so101_perception" / "config" / "wrist_camera_fisheye.yaml"

SQUARES_X, SQUARES_Y = 7, 5
PDF_SQUARE_MM, PDF_MARKER_MM = 25.0, 18.0
DICTIONARY = cv2.aruco.DICT_6X6_250

MIN_CORNERS = 8          # of 24; fewer gives a weak, often degenerate view
OUTLIER_FACTOR = 3.0     # views above this times the median error are dropped
OUTLIER_FLOOR_PX = 1.0   # ... but never one below this


def make_board(square_mm):
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    marker_mm = square_mm * PDF_MARKER_MM / PDF_SQUARE_MM
    board = cv2.aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y, square_mm / 1000, marker_mm / 1000, dictionary)
    return board, dictionary


def detect(gray, board, dictionary):
    """ChArUco corners (N,1,2) and their ids (N,1), or (None, None)."""
    markers, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < 2:
        return None, None
    # Without a camera matrix every corner is interpolated from its two
    # neighbouring markers only, which holds up under strong distortion.
    n, corners, corner_ids = cv2.aruco.interpolateCornersCharuco(markers, ids, gray, board)
    if not n:
        return None, None
    return corners, corner_ids


def open_camera(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok:
        sys.exit(f"no frame from {device}")
    if frame.shape[:2] != (HEIGHT, WIDTH):
        sys.exit(f"camera gives {frame.shape[1]}x{frame.shape[0]}, expected {WIDTH}x{HEIGHT}")
    return cap


def read(cap):
    ok, frame = cap.read()
    if ok and ROTATE_180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    return ok, frame


def cmd_capture(args):
    board, dictionary = make_board(args.square_mm)
    args.images.mkdir(parents=True, exist_ok=True)
    count = len(list(args.images.glob("*.png")))
    coverage = np.zeros((HEIGHT, WIDTH, 3), np.uint8)   # every saved corner, to spot empty areas
    cap = open_camera(args.device)
    cv2.namedWindow("capture", cv2.WINDOW_NORMAL)
    print(f"saving to {args.images} ({count} frames there already)")
    print("SPACE save, q quit. Cover the whole image, edges and corners most of all, tilt the board up to ~45 deg.")
    while True:
        ok, frame = read(cap)
        if not ok:
            continue
        corners, ids = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), board, dictionary)
        n = 0 if ids is None else len(ids)
        view = cv2.addWeighted(frame, 1.0, coverage, 0.6, 0)
        if n:
            cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
        color = (0, 255, 0) if n >= MIN_CORNERS else (0, 0, 255)
        cv2.putText(view, f"corners {n}/24   saved {count}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.imshow("capture", view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            if n < MIN_CORNERS:
                print(f"  not saved: {n} corners, need {MIN_CORNERS}")
                continue
            path = args.images / f"frame_{datetime.now():%Y%m%d_%H%M%S_%f}.png"
            cv2.imwrite(str(path), frame)
            for c in corners.reshape(-1, 2):
                cv2.circle(coverage, (int(c[0]), int(c[1])), 6, (255, 128, 0), -1)
            count += 1
            print(f"  {count:3d}  {path.name}  {n} corners")
    cap.release()
    cv2.destroyAllWindows()


def view_errors(obj, img, K, D, rvecs, tvecs):
    errors = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.fisheye.projectPoints(o, r, t, K, D)
        errors.append(float(np.sqrt(((proj - i) ** 2).sum(-1).mean())))
    return np.array(errors)


def fisheye_calibrate(obj, img):
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(obj, img, (WIDTH, HEIGHT), None, None, flags=flags, criteria=criteria)
    return rms, K, D, rvecs, tvecs


def cmd_calibrate(args):
    board, dictionary = make_board(args.square_mm)
    board_points = np.asarray(board.chessboardCorners, np.float64)
    names, obj, img = [], [], []
    for path in sorted(args.images.glob("*.png")):
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray.shape != (HEIGHT, WIDTH):
            print(f"  skip {path.name}: {gray.shape[1]}x{gray.shape[0]}")
            continue
        corners, ids = detect(gray, board, dictionary)
        if ids is None or len(ids) < MIN_CORNERS:
            print(f"  skip {path.name}: {0 if ids is None else len(ids)} corners")
            continue
        names.append(path.name)
        obj.append(board_points[ids.flatten()].reshape(-1, 1, 3))
        img.append(corners.astype(np.float64).reshape(-1, 1, 2))
    print(f"{len(names)} usable frames in {args.images}")
    if len(names) < 10:
        sys.exit("need at least 10, better 25-40")

    while True:
        try:
            rms, K, D, rvecs, tvecs = fisheye_calibrate(obj, img)
        except cv2.error as e:
            sys.exit(f"cv2.fisheye.calibrate failed: {e}")
        errors = view_errors(obj, img, K, D, rvecs, tvecs)
        limit = max(OUTLIER_FACTOR * np.median(errors), OUTLIER_FLOOR_PX)
        bad = np.flatnonzero(errors > limit)
        if not len(bad) or len(names) - len(bad) < 10:
            break
        for k in sorted(bad, reverse=True):
            print(f"  drop {names[k]}: {errors[k]:.2f} px (limit {limit:.2f})")
            del names[k], obj[k], img[k]

    print("\nper frame error [px]:")
    for name, e in zip(names, errors):
        print(f"  {e:5.2f}  {name}")
    print(f"\nRMS {rms:.3f} px over {len(names)} frames")
    print(f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}")
    print(f"k1..k4 {np.array2string(D.flatten(), precision=5)}")

    P = np.hstack([K, np.zeros((3, 1))])
    data = {
        "image_width": WIDTH,
        "image_height": HEIGHT,
        "camera_name": "so101_wrist",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in K.flatten()]},
        "distortion_model": "equidistant",
        "distortion_coefficients": {"rows": 1, "cols": 4, "data": [float(v) for v in D.flatten()]},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in np.eye(3).flatten()]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": [float(v) for v in P.flatten()]},
    }
    args.file.parent.mkdir(parents=True, exist_ok=True)
    with open(args.file, "w") as f:
        f.write(f"# so101_camera_calib.py, {datetime.now():%Y-%m-%d %H:%M}\n")
        f.write(f"# RMS {rms:.3f} px, {len(names)} frames, ChArUco {SQUARES_X}x{SQUARES_Y} "
                f"square {args.square_mm} mm\n")
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    print(f"saved {args.file}")


def cmd_view(args):
    with open(args.file) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(data["distortion_coefficients"]["data"]).reshape(4, 1)
    size = (data["image_width"], data["image_height"])
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, size, np.eye(3), balance=args.balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, size, cv2.CV_16SC2)
    cap = open_camera(args.device)
    cv2.namedWindow("raw | undistorted", cv2.WINDOW_NORMAL)
    print(f"balance {args.balance} (0 = crop to valid pixels, 1 = keep the whole field of view). q quits.")
    while True:
        ok, frame = read(cap)
        if not ok:
            continue
        undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        cv2.imshow("raw | undistorted", np.hstack([frame, undistorted]))
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="V4L2 device (default: the USB2.0 wrist camera)")
    parser.add_argument("--images", type=Path, default=DEFAULT_IMAGES, help=f"frame folder (default: {DEFAULT_IMAGES})")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"result YAML (default: {DEFAULT_FILE})")
    parser.add_argument("--square-mm", type=float, default=24.0, help="measured printed square side (default: 24)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("capture")
    sub.add_parser("calibrate")
    view = sub.add_parser("view")
    view.add_argument("--balance", type=float, default=0.0, help="0..1, see the printed hint")
    args = parser.parse_args()
    {"capture": cmd_capture, "calibrate": cmd_calibrate, "view": cmd_view}[args.cmd](args)


if __name__ == "__main__":
    main()
