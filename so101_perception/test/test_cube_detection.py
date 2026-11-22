"""Synthetic check of cube_detection: a cube at a known pose is ray-traced through the wrist camera's
fisheye model, detected, and its centre and yaw compared with the truth.

    python3 -m pytest so101_perception/test/test_cube_detection.py -q
"""

import os
import sys

import cv2
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import cube_detection as cd  # noqa: E402

CALIB = os.path.join(HERE, "..", "config", "wrist_camera_fisheye.yaml")
W, H = 640, 360   # the published size (scale 0.5)


def camera():
    with open(CALIB) as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
    return cd.scaled_camera(K, W), D


def marker_texture(marker_id, cells=6, px_per_cell=20):
    """Marker with its one-cell black border and one white cell around it (the quiet zone)."""
    side = cells * px_per_cell
    d = cv2.aruco.getPredefinedDictionary(cd.DICTIONARY)
    img = cv2.aruco.generateImageMarker(d, marker_id, side) if hasattr(cv2.aruco, "generateImageMarker") \
        else cv2.aruco.drawMarker(d, marker_id, side)
    return cv2.copyMakeBorder(img, px_per_cell, px_per_cell, px_per_cell, px_per_cell, cv2.BORDER_CONSTANT, value=255)


def face_frames(yaw):
    """Rotation (face -> cube) of the six faces, z out of the face; then the cube rotated by yaw."""
    Rz = cv2.Rodrigues(np.array([0, 0, yaw]))[0]
    faces = []
    for axis, sign in [(2, 1), (2, -1), (0, 1), (0, -1), (1, 1), (1, -1)]:
        z = np.zeros(3)
        z[axis] = sign
        x = np.array([1.0, 0, 0]) if axis != 0 else np.array([0, 1.0, 0])
        y = np.cross(z, x)
        faces.append(Rz @ np.c_[x, y, z])
    return faces


def render(cube_k, centre_w, yaw, R_wc, t_wc, K, D):
    """Ray-trace the cube into a grey W x H image with a mid-grey table background."""
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    px = np.c_[u.ravel(), v.ravel()].reshape(-1, 1, 2)
    xy = cv2.fisheye.undistortPoints(px, K, D).reshape(-1, 2)
    rays_c = np.c_[xy, np.ones(len(xy))]
    rays_w = rays_c @ R_wc.T
    img = np.full(len(xy), 120, np.uint8)
    depth = np.full(len(xy), np.inf)
    half = cd.CUBE_EDGE / 2
    for f, R_f in enumerate(face_frames(yaw)):
        tex = marker_texture(6 * cube_k + f)
        n = R_f[:, 2]
        p0 = centre_w + half * n
        denom = rays_w @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            s = ((p0 - t_wc) @ n) / denom
        hit = (denom < 0) & (s > 0) & (s < depth)
        local = ((t_wc + s[:, None] * rays_w) - p0) @ R_f        # face coordinates
        a, b = local[:, 0], local[:, 1]
        hit &= (np.abs(a) <= half) & (np.abs(b) <= half)
        # texture covers the marker (22 mm) plus its quiet zone; the cube face beyond is white
        span = cd.MARKER_SIZE * 8 / 6
        tu = ((a / span) + 0.5) * tex.shape[1]
        tv = ((-b / span) + 0.5) * tex.shape[0]
        inside = (tu >= 0) & (tu < tex.shape[1]) & (tv >= 0) & (tv < tex.shape[0])
        val = np.full(len(xy), 255, np.uint8)
        ok = hit & inside
        val[ok] = tex[tv[ok].astype(int), tu[ok].astype(int)]
        img[hit] = val[hit]
        depth[hit] = s[hit]
    return img.reshape(H, W)


def look_at(eye, target):
    """Camera (optical frame: z forward, y down) at eye looking at target, x roughly along world -y."""
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.c_[x, y, z], eye


@pytest.mark.parametrize("cube_k,yaw_deg,eye", [
    (0, 0.0, [0.20, 0.0, 0.25]),
    (3, 25.0, [0.15, 0.05, 0.22]),
    (5, -40.0, [0.30, -0.08, 0.18]),
])
def test_centre_and_yaw(cube_k, yaw_deg, eye):
    K, D = camera()
    centre = np.array([0.25, 0.03, cd.CUBE_EDGE / 2])
    yaw = np.radians(yaw_deg)
    R_wc, t_wc = look_at(np.array(eye), centre)
    gray = render(cube_k, centre, yaw, R_wc, t_wc, K, D)

    poses = cd.marker_poses(cd.MarkerDetector().detect(gray), K, D)
    free = cd.cube_estimates(poses, R_wc, t_wc)
    assert [e.cube for e in free] == [cube_k], f"detected {[p.marker_id for p in poses]}"
    assert np.linalg.norm(free[0].centre - centre) < 0.006, (free[0].centre, centre)
    # resting on the table: the height is known, the rest comes from the viewing rays
    e = cd.cube_estimates(poses, R_wc, t_wc, table_centre_z=centre[2])[0]
    assert np.linalg.norm(e.centre - centre) < 0.0025, (e.centre, centre)
    assert abs(cd.fold_yaw(e.yaw - yaw)) < np.radians(3)
    assert e.up_axis_tilt < np.radians(5)


def test_fold_yaw():
    for a in np.radians([0, 44, 46, 90, 135, -100, 181]):
        f = cd.fold_yaw(a)
        assert -np.pi / 4 - 1e-12 <= f < np.pi / 4
        assert abs(np.sin(4 * (f - a))) < 1e-9


def test_foreign_ids_ignored():
    assert cd.cube_of(35) == 5
    assert cd.cube_of(36) is None
