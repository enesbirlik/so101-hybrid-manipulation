"""Marker-less detection of plain wooden boxes (cubes of unknown, differing sizes) on a white table.

No ROS; numpy, OpenCV and scipy. Per frame:

1. Segmentation (`wood_mask`): wood pixels by HSV (beige hue, medium saturation). The white table
   (low saturation), grey shadows (low saturation, reddish hue), the orange gripper jaws and the
   dark brown desk (both too saturated) fall out. Optionally only pixels whose ray hits the table
   plane inside a work area (`work_area_mask`) are kept: that removes the room and the horizon.
2. Blobs (`split_blobs`): connected components; touching boxes are split by a watershed on the
   distance transform of the mask.
3. Box fit (`fit_box`): an upright box (centre x, y, yaw, edges a, b, height h) standing on the
   table (bottom at table_z) is projected through the fisheye model; the convex hull of its 8
   projected corners is its silhouette. Least squares (scipy, soft-L1) on a symmetric contour
   distance: blob contour points to the silhouette polygon and silhouette samples to the blob
   contour, plus a weak "near cube" prior (a ~ b ~ h). Started from the blob's footprint
   (contour back-projected to the table at mid height, minimum-area rectangle). Two yaw starts.
4. Quality: IoU of the fitted silhouette with the blob, and the contour RMS in pixels.

Limitations: one box per blob after the split (boxes that overlap in the image, one in front of
the other, are split badly or merged; their fits then fail the IoU test and are dropped); a box
sitting on another box is fitted as a taller box standing on the table; seen straight from above,
height and footprint trade off against each other and only the cube prior separates them.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

from cube_detection import fold_yaw


@dataclass
class WoodParams:
    # HSV thresholds, OpenCV ranges (H 0..180)
    h_min: int = 4
    h_max: int = 26
    s_min: int = 30          # wood S 105..150 in normal light but only 35..55 overexposed (2026-11-27 frame 2); the table is rejected by hue (H ~110-146), not by S
    s_max: int = 172         # desk ~195, orange jaws ~230
    v_min: int = 45
    dark_v: int = 125         # below this brightness a pixel must be more saturated to count as wood:
    bright_v: int = 150       # pixels at least this bright need only bright_s_min
    bright_s_min: int = 15
    dark_s_min: int = 90      # dark pixels (V < dark_v) need this S: table shadows S 47..66 (some 80..97), wood in the arm's shadow S 100..114, dark wood sides 140..150
    open_px: float = 0.004   # morphology kernel as a fraction of the image width
    min_area: float = 2.5e-4  # blob area as a fraction of the image area
    split_frac: float = 0.55  # watershed seeds: distance transform above this fraction of the blob's max
    # work area on the table, base_link (m)
    x_range: tuple = (0.05, 0.55)
    y_range: tuple = (-0.40, 0.40)
    # box fit
    size_range: tuple = (0.012, 0.075)
    cube_prior: float = 0.15  # px of residual per mm of (a - b), (h - mean(a, b))
    contour_pts: int = 160
    hull_pts: int = 120
    top_pts: int = 60         # samples on the top face outline, matched to image edges
    top_weight: float = 2.0
    canny: tuple = (25, 70)
    orange_s_min: int = 185            # jaw orange (S ~230) is grown by orange_grow and cut out of the mask; 0 = off
    orange_grow: float = 0.02          # of the image width
    min_footprint: float = 0.016       # m: blobs whose mid-height footprint is smaller on both sides are not fitted
    max_aspect: float = 1.5            # longer / shorter horizontal edge above this = merged cubes, rejected
    border_px: float = 2.0             # contour points this close to the image border are not silhouette edges
    collapse_frac: float = 0.6         # fitted height below this x the shorter footprint edge: refit as a near-cube
    collapse_prior: float = 2.0        # cube prior weight of that refit
    collapse_rms_px: float = 0.8       # the near-cube refit is taken when its contour RMS is at most this much worse
    cache_iou: float = 0.97            # detect_boxes(cache=...): reuse the previous fit of a blob this alike
    workers: int = 4                   # processes fitting blobs in parallel (<= 1: in this process); 10 kept the load at 17-20 on 12 cores (2026-11-27) next to ros2_control
    start_nfev: int = 25                # max evaluations of each yaw start before the best is refined
    yaw_starts: int = 4
    outline_px: float = 1.2   # the blob outline is moved out by this: 0.5 px (pixel centres) + ~0.7 px the
    #                           thresholds lose on the blurred edge (synthetic test; check on real frames)
    min_iou: float = 0.80
    max_rms_px: float = 3.0


@dataclass
class WoodBox:
    centre: np.ndarray        # m, base_link; z = bottom + h/2
    yaw: float                # rad, folded into [-pi/4, pi/4)
    size: np.ndarray          # m: (edge along the yaw axis, the other horizontal edge, height)
    iou: float
    rms_px: float
    confidence: float
    contour: np.ndarray = None    # blob contour, px
    silhouette: np.ndarray = None  # fitted hull, px


# -- camera ---------------------------------------------------------------------------------
def pixel_rays(K, D, width, height):
    """Unit rays (camera frame) of every pixel centre -> (h, w, 3)."""
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    xy = cv2.fisheye.undistortPoints(np.c_[u.ravel(), v.ravel()].reshape(-1, 1, 2), K, D).reshape(-1, 2)
    r = np.c_[xy, np.ones(len(xy))]
    r /= np.linalg.norm(r, axis=1, keepdims=True)
    return r.reshape(height, width, 3)


def project(points_w, R_wc, t_wc, K, D):
    """World points (n, 3) -> pixels (n, 2); points behind the camera are pushed just in front."""
    pc = (np.asarray(points_w, float) - t_wc) @ R_wc
    pc[:, 2] = np.maximum(pc[:, 2], 1e-3)
    px, _ = cv2.fisheye.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    return px.reshape(-1, 2)


def backproject(px, R_wc, t_wc, K, D, z):
    """Pixels (n, 2) -> points on the plane z (n, 3); NaN where the ray does not go down to it."""
    xy = cv2.fisheye.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays = np.c_[xy, np.ones(len(xy))] @ R_wc.T
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (z - t_wc[2]) / rays[:, 2]
    s = np.where(s > 0, s, np.nan)
    return t_wc + s[:, None] * rays


def work_area_mask(rays_cam, R_wc, t_wc, table_z, prm):
    """Pixels whose ray hits the table inside the work area (rays_cam: pixel_rays())."""
    rays = rays_cam @ R_wc.T
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (table_z - t_wc[2]) / rays[..., 2]
    ok = s > 0
    x = t_wc[0] + s * rays[..., 0]
    y = t_wc[1] + s * rays[..., 1]
    ok &= (x >= prm.x_range[0]) & (x <= prm.x_range[1]) & (y >= prm.y_range[0]) & (y <= prm.y_range[1])
    return ok


# -- segmentation ---------------------------------------------------------------------------
def wood_mask(bgr, prm=WoodParams(), roi=None):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # S threshold by brightness: pale, lit wood sides are bright (V ~180) but nearly grey (S 25-35),
    # table shadows are darker (V 95-140) with S 45-66; the white table itself is S < 10
    m = cv2.inRange(hsv, (prm.h_min, min(prm.s_min, prm.bright_s_min), prm.v_min), (prm.h_max, prm.s_max, 255))
    V, Sat = hsv[..., 2], hsv[..., 1]
    m[(V < prm.bright_v) & (Sat < prm.s_min)] = 0
    m[(V < prm.dark_v) & (Sat < prm.dark_s_min)] = 0
    if prm.orange_s_min > 0:
        # the orange jaws: their blurred edges fall into the wood range (real arm 2026-11-27, LOOK pose)
        orange = cv2.inRange(hsv, (0, prm.orange_s_min, 60), (prm.h_max, 255, 255))
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        k = max(3, int(round(prm.orange_grow * bgr.shape[1])) | 1)
        m[cv2.dilate(orange, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))) > 0] = 0
    if roi is not None:
        m[~roi] = 0
    k = max(3, int(round(prm.open_px * bgr.shape[1])) | 1)
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker)
    return m


def _fill_holes(m):
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = np.zeros_like(m)
    cv2.drawContours(out, cnts, -1, 255, -1)
    return out


def split_blobs(mask, bgr, prm=WoodParams()):
    """Mask -> list of single-blob masks (uint8), touching boxes split by a watershed."""
    mask = _fill_holes(mask)
    min_area = prm.min_area * mask.size
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            continue
        comp = (lab == i).astype(np.uint8) * 255
        dist = cv2.distanceTransform(comp, cv2.DIST_L2, 5)
        seeds = (dist > prm.split_frac * dist.max()).astype(np.uint8)
        ns, seed_lab = cv2.connectedComponents(seeds)
        if ns <= 2:
            out.append(comp)
            continue
        markers = seed_lab.astype(np.int32)
        markers[comp == 0] = ns          # background label
        cv2.watershed(bgr, markers)
        for j in range(1, ns):
            part = ((markers == j) & (comp > 0)).astype(np.uint8) * 255
            part = cv2.morphologyEx(part, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            if cv2.countNonZero(part) >= min_area:
                out.append(part)
    return out


# -- box fit --------------------------------------------------------------------------------
_UNIT_BOX = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (0.0, 1.0)])


_EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]
_TOP = [1, 3, 7, 5]   # the 4 top corners of _UNIT_BOX in polygon order
_STEPS = np.linspace(0, 1, 7)[:-1]
# 12 edges x 6 points: straight 3D edges are curves in the fisheye image, so the hull is taken of these
_UNIT_DENSE = np.concatenate([_UNIT_BOX[i] + _STEPS[:, None] * (_UNIT_BOX[j] - _UNIT_BOX[i]) for i, j in _EDGES]
                             + [_UNIT_BOX])
_UNIT_TOP = np.concatenate([_UNIT_BOX[i] + _STEPS[:, None] * (_UNIT_BOX[j] - _UNIT_BOX[i])
                            for i, j in zip(_TOP, _TOP[1:] + _TOP[:1])])


def _place(unit, p, z0):
    cx, cy, yaw, a, b, h = p
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return (unit * [a, b, h]) @ R.T + [cx, cy, z0]


def box_corners(p, z0):
    return _place(_UNIT_BOX, p, z0)


def silhouette(p, z0, R_wc, t_wc, K, D):
    px = project(_place(_UNIT_DENSE, p, z0), R_wc, t_wc, K, D).astype(np.float32)
    return cv2.convexHull(px).reshape(-1, 2).astype(np.float64)


def _sample_polygon(P, n):
    seg = np.roll(P, -1, axis=0) - P
    L = np.linalg.norm(seg, axis=1)
    cum = np.r_[0, np.cumsum(L)]
    t = (np.arange(n) + 0.5) / n * cum[-1]
    i = np.clip(np.searchsorted(cum, t, side="right") - 1, 0, len(P) - 1)
    f = (t - cum[i]) / np.maximum(L[i], 1e-9)
    return P[i] + f[:, None] * seg[i]


def _dist_to_polygon(Q, P):
    A, B = P, np.roll(P, -1, axis=0)
    AB = B - A
    t = np.clip(np.einsum("qij,ij->qi", Q[:, None, :] - A[None], AB) / np.maximum((AB ** 2).sum(1), 1e-9), 0, 1)
    closest = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(Q[:, None, :] - closest, axis=2).min(axis=1)


def _residuals(p, z0, contour, cam, prm, edge_dt=None, border=None, cube_prior=None):
    """border: (contour points on the image border (bool), width, height) or None. A cube cut by the
    image border: its outline along the border is not a silhouette edge, and the silhouette may run
    on outside the image: neither counts."""
    R_wc, t_wc, K, D = cam
    P = silhouette(p, z0, R_wc, t_wc, K, D)
    n_top = prm.top_pts if edge_dt is not None else 0
    if len(P) < 3:
        return np.full(len(contour) + prm.hull_pts + n_top + 2, 50.0)
    r1 = _dist_to_polygon(contour, P)
    S = _sample_polygon(P, prm.hull_pts)
    r2 = np.sqrt(((S[:, None, :] - contour[None]) ** 2).sum(2)).min(axis=1)
    if border is not None:
        on_border, W, H = border
        r1 = np.where(on_border, 0.0, r1)
        m = prm.border_px
        r2 = np.where((S[:, 0] < m) | (S[:, 1] < m) | (S[:, 0] > W - 1 - m) | (S[:, 1] > H - 1 - m), 0.0, r2)
    a, b, h = p[3:] * 1000
    cp = prm.cube_prior if cube_prior is None else cube_prior
    prior = cp * np.array([a - b, h - (a + b) / 2])
    if edge_dt is None:
        return np.r_[r1, r2, prior]
    T = _sample_polygon(project(_place(_UNIT_TOP, p, z0), R_wc, t_wc, K, D), n_top)
    r3 = prm.top_weight * map_coordinates(edge_dt, [T[:, 1], T[:, 0]], order=1, mode="nearest")
    return np.r_[r1, r2, r3, prior]


def edge_distance(gray, mask, prm):
    """Distance (px) to the nearest image edge near the blob: the top face's outline."""
    g = cv2.GaussianBlur(gray, (0, 0), 1.0)
    e = cv2.Canny(g, *prm.canny)
    e[cv2.dilate(mask, np.ones((5, 5), np.uint8)) == 0] = 0
    return cv2.distanceTransform(255 - e, cv2.DIST_L2, 3).astype(np.float64)


def _contour(mask, n, offset=0.5):
    """Blob outline, n points. findContours gives the centres of the boundary pixels, half a pixel
    inside the true edge (and the thresholds lose a bit more): every point is moved out by
    `offset` px along the outline's normal."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    if len(c) >= 8:
        tang = np.roll(c, -2, axis=0) - np.roll(c, 2, axis=0)
        nrm = np.c_[tang[:, 1], -tang[:, 0]]
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
        if cv2.contourArea(c.astype(np.float32), oriented=True) > 0:
            nrm = -nrm
        # outward: the side away from the blob's centroid on average
        if np.mean(np.sum((c - c.mean(axis=0)) * nrm, axis=1)) < 0:
            nrm = -nrm
        c = c + offset * nrm
    return c[np.linspace(0, len(c), n, endpoint=False).astype(int)] if len(c) > n else c


def initial_box(contour, cam, z0, prm, h0=0.03):
    R_wc, t_wc, K, D = cam
    w = backproject(contour, R_wc, t_wc, K, D, z0 + h0 / 2)
    w = w[np.isfinite(w[:, 0])]
    if len(w) < 5:
        return None
    (cx, cy), (a, b), ang = cv2.minAreaRect((w[:, :2] * 1000).astype(np.float32))
    lo, hi = prm.size_range
    if max(a, b) / 1000 < prm.min_footprint:
        return None          # a speck (shadow edge, jaw rim): not worth a fit
    a, b = np.clip(np.array([a, b]) / 1000 * 0.85, lo * 1.05, hi * 0.95)
    h = float(np.clip(np.sqrt(a * b), lo * 1.05, hi * 0.95))
    return np.array([cx / 1000, cy / 1000, np.radians(ang), a, b, h])


def _iou(mask, P):
    sil = np.zeros_like(mask)
    cv2.fillConvexPoly(sil, np.round(P).astype(np.int32), 255)
    inter = np.count_nonzero((sil > 0) & (mask > 0))
    union = np.count_nonzero((sil > 0) | (mask > 0))
    return inter / max(union, 1), sil


def fit_box(mask, R_wc, t_wc, K, D, table_z, prm=WoodParams(), gray=None):
    """One blob mask (+ the grey image for the top face's edges) -> WoodBox or None."""
    contour = _contour(mask, prm.contour_pts, prm.outline_px)
    edge_dt = edge_distance(gray, mask, prm) if gray is not None and prm.top_weight > 0 else None
    cam = (R_wc, t_wc, K, D)
    H_img, W_img = mask.shape[:2]
    m = prm.border_px
    on_border = (contour[:, 0] < m) | (contour[:, 1] < m) | (contour[:, 0] > W_img - 1 - m) | (contour[:, 1] > H_img - 1 - m)
    border = (on_border, W_img, H_img) if on_border.any() else None
    p0 = initial_box(contour, cam, table_z, prm)
    if p0 is None:
        return None
    lo, hi = prm.size_range
    lb = [-np.inf, -np.inf, -np.inf, lo, lo, lo]
    ub = [np.inf, np.inf, np.inf, hi, hi, hi]
    kw = dict(args=(table_z, contour, cam, prm, edge_dt, border), bounds=(lb, ub), loss="soft_l1", f_scale=2.0,
              x_scale=[0.005, 0.005, 0.1, 0.005, 0.005, 0.005])
    best = None
    # short fits from every yaw start, then only the best one runs to convergence (~3x faster)
    for dyaw in np.arange(prm.yaw_starts) * (np.pi / 2 / prm.yaw_starts):
        p = p0.copy()
        p[2] += dyaw
        try:
            r = least_squares(_residuals, p, max_nfev=prm.start_nfev, **kw)
        except ValueError:
            continue
        if best is None or r.cost < best.cost:
            best = r
    if best is None:
        return None
    if not best.success or best.nfev >= prm.start_nfev:
        try:
            best = least_squares(_residuals, best.x, max_nfev=200, **kw)
        except ValueError:
            return None
    n = len(contour) + prm.hull_pts   # silhouette residuals only
    a_, b_, h_ = best.x[3:]
    if h_ < prm.collapse_frac * min(a_, b_) or h_ < lo * 1.05:
        # height collapsed (seen from nearly straight above, a flat box fits the outline as well as a
        # cube; real arm 2026-11-27: 30 mm cubes came out 12 mm tall): fit again as a near-cube
        q = best.x.copy()
        q[5] = float(np.clip((a_ + b_) / 2, lo * 1.05, hi * 0.95))
        kw2 = dict(kw, args=(table_z, contour, cam, prm, edge_dt, border, prm.collapse_prior))
        try:
            r = least_squares(_residuals, q, max_nfev=200, **kw2)
            if np.sqrt(np.mean(r.fun[:n] ** 2)) < np.sqrt(np.mean(best.fun[:n] ** 2)) + prm.collapse_rms_px:
                best = r
        except ValueError:
            pass
    p = best.x
    rms = float(np.sqrt(np.mean(best.fun[:n] ** 2)))
    P = silhouette(p, table_z, *cam)
    iou, _ = _iou(mask, P)
    yaw = float(fold_yaw(p[2]))
    a, b, h = p[3:]
    if int(round((p[2] - yaw) / (np.pi / 2))) % 2:
        a, b = b, a
    conf = float(np.clip((iou - 0.7) / 0.25, 0, 1) * np.exp(-rms ** 2 / (2 * 1.5 ** 2)))
    return WoodBox(np.array([p[0], p[1], table_z + h / 2]), yaw, np.array([a, b, h]), float(iou), rms, conf,
                   contour, P)


def accepted(box, prm=WoodParams()):
    lo, hi = prm.size_range
    at_bound = np.any(box.size < lo * 1.02) or np.any(box.size > hi * 0.98)
    a, b = box.size[:2]
    merged = max(a, b) / max(min(a, b), 1e-6) > prm.max_aspect   # two touching cubes fitted as one long box
    return box.iou >= prm.min_iou and box.rms_px <= prm.max_rms_px and not at_bound and not merged


def detect_boxes(bgr, R_wc, t_wc, K, D, table_z, prm=WoodParams(), rays_cam=None, roi=None, keep_rejected=False,
                 cache=None):
    """Image + camera pose -> (accepted WoodBoxes, rejected WoodBoxes, mask).

    rays_cam: pixel_rays() of this image size (cached by the caller) to restrict to the work area.
    cache: a dict the caller keeps between frames: a blob whose mask matches one of the previous
    frame (IoU >= prm.cache_iou) seen from the same camera pose (within 1 mm / 0.2 deg) reuses that
    frame's fit instead of fitting again (the arm stands still at a view pose: frames are alike).
    """
    area = roi
    if rays_cam is not None:
        wa = work_area_mask(rays_cam, R_wc, t_wc, table_z, prm)
        area = wa if area is None else (area & wa)
    mask = wood_mask(bgr, prm, area)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    good, bad = [], []
    blobs = split_blobs(mask, bgr, prm)
    boxes = [None] * len(blobs)
    todo = list(range(len(blobs)))
    same_pose = False
    if cache is not None and "pose" in cache:
        R0, t0 = cache["pose"]
        dang = np.degrees(np.arccos(np.clip((np.trace(R0.T @ R_wc) - 1) / 2, -1, 1)))
        same_pose = np.linalg.norm(t0 - t_wc) < 1e-3 and dang < 0.2
    if same_pose:
        todo = []
        for i, b in enumerate(blobs):
            hit = _cached(cache["blobs"], b, prm.cache_iou)
            if hit is not None:
                boxes[i] = hit
            else:
                todo.append(i)
    ex = _executor(prm.workers) if len(todo) > 1 else None
    args = [(blobs[i], R_wc, t_wc, K, D, table_z, prm, gray) for i in todo]
    try:
        fitted = list(ex.map(_fit_safe, args)) if ex is not None else [_fit_safe(a) for a in args]
    except Exception:   # broken pool (a worker died): drop it, fit here; the next frame forks a new one
        _EXECUTOR.pop(prm.workers, None)
        fitted = [_fit_safe(a) for a in args]
    for i, box in zip(todo, fitted):
        boxes[i] = box
    # touching cubes that the watershed left as one blob fit as one long box: split it into
    # round(long / short) parts along its pixels (k-means) and fit each (real arm 2026-11-27)
    split_args, split_of = [], []
    for i, box in enumerate(boxes):
        if box is None or isinstance(box, list):
            continue
        a, b = sorted(box.size[:2])
        if b / max(a, 1e-6) > prm.max_aspect:
            for k in (2, 3):
                for part in _split_k(blobs[i], k):
                    split_args.append((part, R_wc, t_wc, K, D, table_z, prm, gray))
                    split_of.append((i, k))
    if split_args:
        ex = ex or _executor(prm.workers)
        try:
            parts = list(ex.map(_fit_safe, split_args)) if ex is not None else [_fit_safe(a) for a in split_args]
        except Exception:
            parts = [_fit_safe(a) for a in split_args]
        for i in sorted({j for j, _ in split_of}):
            best = None
            for k in (2, 3):
                mine = [p for p, jk in zip(parts, split_of) if jk == (i, k)]
                good = [p for p in mine if p is not None and accepted(p, prm)]
                # all parts good beats some; then more good parts; then the smaller k
                key = (len(good) == len(mine), len(good), -k)
                if good and (best is None or key > best[0]):
                    best = (key, good)
            if best is not None:
                boxes[i] = best[1]     # this blob -> several boxes
    if cache is not None:
        cache["pose"] = (R_wc.copy(), np.asarray(t_wc, float).copy())
        cache["blobs"] = [(b, box) for b, box in zip(blobs, boxes)]
    flat = []
    for box in boxes:
        flat.extend(box if isinstance(box, list) else [box])
    boxes = flat
    for box in boxes:
        if box is None:
            continue
        (good if accepted(box, prm) else bad).append(box)
    return good, bad, mask


def _cached(entries, blob, min_iou):
    for b, box in entries:
        inter = np.count_nonzero(b & blob) if b.dtype == bool else np.count_nonzero((b > 0) & (blob > 0))
        union = np.count_nonzero((b > 0) | (blob > 0))
        if union and inter / union >= min_iou:
            return box
    return None


def _split_k(blob, k):
    """Blob mask -> k masks: k-means of its pixel coordinates (touching cubes of about equal size)."""
    ys, xs = np.nonzero(blob)
    pts = np.c_[xs, ys].astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.5)
    _, lab, _ = cv2.kmeans(pts, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
    out = []
    for j in range(k):
        m = np.zeros_like(blob)
        sel = lab.ravel() == j
        m[ys[sel], xs[sel]] = 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        out.append(m)
    return out


def _fit_safe(args):
    try:
        return fit_box(*args)
    except (cv2.error, np.linalg.LinAlgError, ValueError):
        return None


_EXECUTOR = {}


def _executor(workers):
    """One process pool per worker count (spawned once, reused every frame); None for workers <= 1.
    The box fits are independent and pure Python/numpy bound: 17 blobs took 15 s on one core
    (real LOOK frame 2026-11-27)."""
    if workers <= 1:
        return None
    if workers not in _EXECUTOR:
        import multiprocessing as mp
        from concurrent.futures import ProcessPoolExecutor
        _EXECUTOR[workers] = ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn"))   # not fork: the ROS node is multi-threaded
    return _EXECUTOR[workers]


# -- tracking -------------------------------------------------------------------------------
class WoodTracker:
    """Persistent ids for marker-less boxes: an observation joins the nearest track whose centre is
    within `assoc_mm` (each track takes at most one box per frame); otherwise it starts a new id
    (first_id, first_id + 1, ...). Pose filtering and the stable flag come from cube_tracking.CubeTrack;
    the size is the median of the last `window` observations."""

    def __init__(self, params=None, assoc_mm=20.0, first_id=10):
        from cube_tracking import TrackerParams
        self.p = params or TrackerParams()
        self.assoc = assoc_mm / 1000
        self.next_id = first_id
        self.tracks = {}      # id -> CubeTrack
        self.sizes = {}       # id -> deque of (a, b, h)
        # forgotten tracks: id -> (last centre, time). A box that turns up where a forgotten one was
        # gets its id back: the pick node keeps asking for the id it was given, and the arm's view
        # poses easily keep a cube out of sight longer than the track memory (real arm 2026-11-27:
        # ids ran up to cube_69 for 8 cubes, and a pick searched 158 s for cube_62 sitting in place).
        self.retired = {}
        self.retired_s = 3600.0

    def update(self, stamp, boxes):
        """-> list of (id, box) for this frame."""
        from collections import deque

        from cube_tracking import CubeTrack, Observation
        pairs = []
        for i, box in enumerate(boxes):
            for k, tr in self.tracks.items():
                if tr.centre is not None:
                    pairs.append((float(np.linalg.norm(tr.centre[:2] - box.centre[:2])), i, k))
        pairs.sort()
        assigned, used = {}, set()
        for d, i, k in pairs:
            if d > self.assoc or i in assigned or k in used:
                continue
            assigned[i] = k
            used.add(k)
        out = []
        for i, box in enumerate(boxes):
            k = assigned.get(i)
            if k is None:
                k = self._revive(box.centre, stamp)
            if k is None:
                k = self.next_id
                self.next_id += 1
            if k not in self.tracks:
                self.tracks[k] = CubeTrack(self.p)
                self.sizes[k] = deque(maxlen=self.p.window)
            tr = self.tracks[k]
            size = box.size.copy()
            # the fold put this box's yaw on the other side of +-45 deg: its a and b are swapped
            if tr.centre is not None and abs(box.yaw - tr.yaw) > np.pi / 4:
                size[:2] = size[1::-1]
            tr.update(_observation(Observation, stamp, box))
            self.sizes[k].append(size)
            out.append((k, box))
        return out

    def size(self, k):
        return np.median(np.array(self.sizes[k]), axis=0)

    def _revive(self, centre, now):
        best, best_d = None, self.assoc
        for k, (c, t) in self.retired.items():
            d = float(np.linalg.norm(c[:2] - centre[:2]))
            if d <= best_d and now - t <= self.retired_s:
                best, best_d = k, d
        if best is not None:
            del self.retired[best]
        return best

    def forget(self, older_than, now):
        gone = [k for k, t in self.tracks.items() if t.age(now) > older_than]
        for k in gone:
            if self.tracks[k].centre is not None:
                self.retired[k] = (np.array(self.tracks[k].centre, float), now)
            del self.tracks[k]
            del self.sizes[k]
        return gone


def _observation(Observation, stamp, box):
    return Observation(stamp, box.centre, box.yaw, box.confidence, 0.0, 0.0)
