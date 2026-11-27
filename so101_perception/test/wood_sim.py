"""Synthetic wrist-camera colour images of wooden boxes on a white table (ray-traced through the
real fisheye calibration), with cast shadows, for the wood detector's tests. No ROS."""

import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import cube_detection as cd  # noqa: E402
import wood_detection as wd  # noqa: E402

CALIB = os.path.join(HERE, "..", "config", "wrist_camera_fisheye.yaml")
W, H = 640, 360


def hsv_bgr(h, s, v):
    return cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0].astype(float)


TABLE = hsv_bgr(146, 32, 170)
SHADOW = hsv_bgr(3, 62, 100)
WOOD_TOP = hsv_bgr(12, 109, 161)
WOOD_LIT = hsv_bgr(11, 141, 120)
WOOD_DARK = hsv_bgr(11, 148, 72)


def camera(width=W):
    with open(CALIB) as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
    return cd.scaled_camera(K, width, c["image_width"]), D


def look_at(eye, target):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, [0, 0, 1.0])
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0, -1.0, 0])
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z], eye


@dataclass
class Box:
    x: float
    y: float
    yaw: float
    a: float
    b: float
    h: float


def _hit_box(o, d, box, z0):
    """Slab test, rays (n,3) from o (n,3 or 3) -> (s, local normal axis index, sign); s = inf on miss."""
    c, s_ = np.cos(box.yaw), np.sin(box.yaw)
    R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
    centre = np.array([box.x, box.y, z0 + box.h / 2])
    ol = (o - centre) @ R
    dl = d @ R
    half = np.array([box.a, box.b, box.h]) / 2
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-half - ol) / dl
        t2 = (half - ol) / dl
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (tmax >= tmin) & (tmax > 1e-6)
    s = np.where(hit, np.where(tmin > 1e-6, tmin, tmax), np.inf)
    axis = np.argmax(np.minimum(t1, t2), axis=1)
    sign = -np.sign(np.take_along_axis(dl, axis[:, None], 1)[:, 0])
    return s, axis, sign, R


def render(boxes, R_wc, t_wc, K, D, table_z=0.0, light=(-0.6, 0.5, 1.0), ss=2, rng=None, width=W, height=H):
    light = np.asarray(light, float) / np.linalg.norm(light)
    u = (np.arange(width * ss) + 0.5) / ss - 0.5
    v = (np.arange(height * ss) + 0.5) / ss - 0.5
    uu, vv = np.meshgrid(u, v)
    xy = cv2.fisheye.undistortPoints(np.c_[uu.ravel(), vv.ravel()].reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays = np.c_[xy, np.ones(len(xy))] @ R_wc.T
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    n = len(rays)
    col = np.tile(TABLE, (n, 1))
    with np.errstate(divide="ignore", invalid="ignore"):
        s_tab = (table_z - t_wc[2]) / rays[:, 2]
    depth = np.where(s_tab > 0, s_tab, np.inf)
    col[~np.isfinite(depth)] = (90, 90, 90)
    face = np.full(n, -1)
    for box in boxes:
        s, axis, sign, R = _hit_box(np.broadcast_to(t_wc, rays.shape), rays, box, table_z)
        closer = s < depth
        depth[closer] = s[closer]
        # world normal of the face hit
        nl = np.zeros((n, 3))
        nl[np.arange(n), axis] = sign
        nw = nl @ R.T
        top = closer & (axis == 2)
        side = closer & (axis != 2)
        lit = side & (nw @ light > 0.2)
        col[top] = WOOD_TOP
        col[side] = WOOD_DARK
        col[lit] = WOOD_LIT
        face[closer] = 1
    # shadows on the table
    table = (face < 0) & np.isfinite(depth)
    if table.any():
        P = t_wc + depth[table, None] * rays[table]
        shadowed = np.zeros(len(P), bool)
        for box in boxes:
            s, *_ = _hit_box(P, np.broadcast_to(light, P.shape), box, table_z)
            shadowed |= np.isfinite(s)
        idx = np.flatnonzero(table)[shadowed]
        col[idx] = SHADOW
    img = col.reshape(height, ss, width, ss, 3).mean(axis=(1, 3))
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    if rng is not None:
        img = img + rng.normal(0, 2.0, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1], cv2.IMREAD_COLOR)


def overview_view():
    """A hand-set wrist pose like the overview pose: ~25 cm above the table, looking down and forward."""
    return look_at([0.12, 0.0, 0.25], [0.24, 0.0, 0.0])
