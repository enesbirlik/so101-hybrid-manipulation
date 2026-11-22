"""ArUco cube detection without ROS: fisheye image -> marker corners -> cube centres and yaw.

The cubes: edge 30 mm, one DICT_4X4_50 marker per face (black square 22 mm), cube k carries ids
6k .. 6k+5 (k = 0..5). Which id sits on which face does not matter: every face's axes are the
cube's axes up to order and sign, so a face's pose tells which face of the cube it is (top or a
side) and how its marker is turned on it.

The cubes are folded from paper: faces are a few degrees off, bulge, and the printed marker sits
a millimetre or two off-centre. So no single face is trusted on its own:

1. Every marker gets both IPPE poses (a small, obliquely seen square has two plausible tilts).
2. A cube resting on the table stands upright: its yaw (mod 90 deg) is the only free angle. The
   face poses propose yaw hypotheses; for each, every face is assigned to the cube face and
   in-plane quarter turn that fits it best.
3. One rigid, upright cube (centre x, y and yaw; z too when it is not on the table) is fitted to
   the corner rays of all its faces at once (Gauss-Newton, Huber-weighted, residuals in pixels).
   With the table prior the cube's height is known, which pins down the distance along the rays,
   the weakest part of a small marker's pose.
4. Faces that do not fit (reprojection, or a centre far from the others) are dropped and the fit
   repeated. Each face is also fitted alone: the spread of these face-implied centres and yaws
   says how consistent the faces are, and goes into the confidence.
5. The yaw comes from the top face when it is seen well enough: its marker edges lie in a
   horizontal plane, so they give the rotation about the vertical directly, while a side face
   gives it only through its out-of-plane tilt.

Pose convention of OpenCV ArUco: marker x right, y up, z out of the face towards the viewer.
"""

import itertools
from dataclasses import dataclass, field

import cv2
import numpy as np

CUBE_EDGE = 0.030
MARKER_SIZE = 0.022
FACES_PER_CUBE = 6
N_CUBES = 6
DICTIONARY = cv2.aruco.DICT_4X4_50

# unit marker corners in the marker frame, ArUco order: top-left, top-right, bottom-right, bottom-left
_UNIT = np.array([[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]], dtype=np.float64)
MARKER_OBJ = _UNIT * MARKER_SIZE


@dataclass(frozen=True)
class CubeGeometry:
    edge: float = CUBE_EDGE        # m
    marker: float = MARKER_SIZE    # m, side of the black square

    @property
    def marker_obj(self):
        return _UNIT * self.marker


def load_cube_geometry(path):
    """cubes.yaml -> {cube k: CubeGeometry}, plus key None for the default.

        default: {edge: 0.030, marker: 0.022}
        cubes:
          3: {edge: 0.0305}          # only what differs
    """
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    default = CubeGeometry(**(data.get("default") or {}))
    out = {None: default}
    for k, g in (data.get("cubes") or {}).items():
        out[int(k)] = CubeGeometry(**{**default.__dict__, **(g or {})})
    return out


def geometry_of(geometry, k):
    """CubeGeometry of cube k from None (nominal), one CubeGeometry, or a load_cube_geometry dict."""
    if geometry is None:
        return CubeGeometry()
    if isinstance(geometry, CubeGeometry):
        return geometry
    return geometry.get(k, geometry.get(None, CubeGeometry()))


def cube_of(marker_id):
    """Cube index of a marker id, or None for ids that belong to no cube."""
    k = marker_id // FACES_PER_CUBE
    return k if 0 <= k < N_CUBES else None


class MarkerDetector:
    """cv2.aruco across the 4.6 (functions) and 4.7+ (ArucoDetector) APIs.

    refinement: "subpix" (cornerSubPix on the corners, default), "contour" (lines fitted to the
    marker's edges), "apriltag" or "none".
    """

    REFINEMENTS = {"none": "CORNER_REFINE_NONE", "subpix": "CORNER_REFINE_SUBPIX",
                   "contour": "CORNER_REFINE_CONTOUR", "apriltag": "CORNER_REFINE_APRILTAG"}

    def __init__(self, refinement="subpix", win_size=3):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(DICTIONARY)
        # 4.6 also exposes DetectorParameters(), but constructing it segfaults there
        params = aruco.DetectorParameters_create() if hasattr(aruco, "DetectorParameters_create") else aruco.DetectorParameters()
        params.cornerRefinementMethod = getattr(aruco, self.REFINEMENTS[refinement])
        params.cornerRefinementWinSize = win_size   # small markers: a wide window slides onto the next cell
        self.params = params
        self.detector = aruco.ArucoDetector(self.dictionary, params) if hasattr(aruco, "ArucoDetector") else None

    def detect(self, gray):
        """-> list of (id, 4x2 corners in pixels)."""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return []
        return [(int(i), c.reshape(4, 2).astype(np.float64)) for i, c in zip(ids.ravel(), corners)]


@dataclass
class MarkerPose:
    marker_id: int
    cube: int
    R: np.ndarray            # marker -> camera rotation (the IPPE solution with the lower error)
    t: np.ndarray            # marker centre in the camera frame, m
    reproj_px: float         # mean corner reprojection error (pixels of the input image)
    corners: np.ndarray      # 4x2 pixels, as detected
    bearings: np.ndarray = None      # 4x3 unit rays of the corners, camera frame
    solutions: list = field(default_factory=list)   # [(R, t, reproj_px)] both IPPE poses, best first
    f_px: float = 0.0        # focal length of the image, px (pixels per radian near the centre)
    geometry: CubeGeometry = CubeGeometry()

    @property
    def cube_centre(self):
        """Cube centre in the camera frame: half an edge behind the face."""
        return self.t + self.R @ np.array([0.0, 0.0, -self.geometry.edge / 2])

    @property
    def side_px(self):
        """Mean side length of the marker in the image, px."""
        return float(np.linalg.norm(self.corners - np.roll(self.corners, -1, axis=0), axis=1).mean())


def marker_poses(detections, K, D, geometry=None):
    """Fisheye (equidistant) K, D of the image the corners come from -> MarkerPose per marker.

    The corners are undistorted to normalised coordinates first, then solved with IPPE_SQUARE
    on an identity camera: solvePnP has no fisheye model. Both IPPE solutions are kept.
    """
    out = []
    for marker_id, px in detections:
        k = cube_of(marker_id)
        if k is None:
            continue
        g = geometry_of(geometry, k)
        obj = g.marker_obj
        px = np.asarray(px, np.float64).reshape(4, 2)
        norm = cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2), K, D).reshape(-1, 2)
        rays = np.c_[norm, np.ones(4)]
        bearings = rays / np.linalg.norm(rays, axis=1, keepdims=True)
        try:
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, norm, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            continue
        sols = []
        for rvec, tvec in zip(rvecs[:n], tvecs[:n]):
            if tvec[2, 0] <= 0:
                continue
            proj, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - px, axis=1).mean())
            sols.append((cv2.Rodrigues(rvec)[0], tvec.ravel().copy(), err))
        if not sols:
            continue
        sols.sort(key=lambda s: s[2])
        R, t, err = sols[0]
        out.append(MarkerPose(marker_id, k, R, t, err, px, bearings, sols, float(K[0, 0]), g))
    return out


@dataclass
class CubeEstimate:
    cube: int
    centre: np.ndarray                 # in the target frame, m
    yaw: float                         # rad in [-pi/4, pi/4): rotation of the cube about the frame's z
    up_axis_tilt: float                # rad: how far the cube's most vertical axis is from z
    faces: list = field(default_factory=list)   # marker ids used
    reproj_px: float = 0.0             # RMS corner residual of the fused fit, px
    confidence: float = 0.0            # 0..1, see cube_estimates()
    sigma_xy: float = 0.0              # m, 1-sigma of the centre from the corner residuals
    sigma_yaw: float = 0.0             # rad
    face_spread: float = 0.0           # m, RMS distance of the face-implied centres from the fused one
    face_spread_yaw: float = 0.0       # rad, RMS of the face-implied yaws about the fused one
    top_face: int = -1                 # marker id of the visible top face, -1 if none
    rejected: list = field(default_factory=list)   # marker ids seen but dropped as outliers
    on_table: bool = False             # the table prior was used (centre height fixed)
    side_px: float = 0.0               # mean marker side in the image, px
    face_centres: dict = field(default_factory=dict)   # marker id -> centre implied by that face alone
    face_yaws: dict = field(default_factory=dict)      # marker id -> yaw implied by that face alone

    @property
    def quaternion(self):
        """x, y, z, w of a pure yaw rotation."""
        return np.array([0.0, 0.0, np.sin(self.yaw / 2), np.cos(self.yaw / 2)])


def fold_yaw(angle):
    """A cube looks the same every 90 deg: fold into [-pi/4, pi/4)."""
    return (angle + np.pi / 4) % (np.pi / 2) - np.pi / 4


def circular_mean_90(angles, weights=None):
    """Mean of yaw angles modulo 90 deg (on the circle of 4x the angle) -> (mean, resultant length 0..1)."""
    angles = np.asarray(angles, float)
    w = np.ones_like(angles) if weights is None else np.asarray(weights, float)
    v = np.array([w @ np.cos(4 * angles), w @ np.sin(4 * angles)]) / max(w.sum(), 1e-12)
    return float(fold_yaw(np.arctan2(v[1], v[0]) / 4)), float(np.linalg.norm(v))


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _cube_rotations():
    """The 24 proper rotations that map the cube onto itself (signed permutation matrices)."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(perm, signs)):
                M[row, col] = s
            if np.linalg.det(M) > 0:
                out.append(M)
    return out


CUBE_ROTATIONS = _cube_rotations()
_CUBE_ROT_ARRAY = np.array(CUBE_ROTATIONS)


def _angle(R):
    return float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


@dataclass
class _Face:
    """One marker of a cube, in the target frame, ready for the fit."""
    pose: MarkerPose
    bearings: np.ndarray      # 4x3 unit rays, target frame
    basis: np.ndarray         # 4x2x3: two unit vectors across each ray (residual directions)
    solutions: list           # [(R_w marker -> target, t_w marker centre, reproj_px)]
    view_cos: float
    weight: float
    S: np.ndarray = None      # marker -> cube frame (one of CUBE_ROTATIONS), once assigned
    local: np.ndarray = None  # 4x3 corners in the cube frame, once assigned
    solution: int = 0         # the IPPE solution that agrees with the assignment


def _ray_basis(b):
    a = np.where(np.abs(b[:, [0]]) < 0.9, np.array([[1.0, 0, 0]]), np.array([[0, 1.0, 0]]))
    u = np.cross(b, a)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.cross(b, u)
    return np.stack([u, v], axis=1)


class _Fit:
    """Upright rigid cube fitted to the corner rays of its faces (analytic Jacobian, all faces at once)."""

    def __init__(self, t_wc, f_px, huber_px=1.0):
        self.t_wc, self.f, self.huber = t_wc, f_px, huber_px

    def _model(self, p, L, B, z):
        """-> residuals (px, 2 per corner) and their Jacobian w.r.t. p = (x, y, yaw[, z])."""
        c, s = np.cos(p[2]), np.sin(p[2])
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        dR = np.array([[-s, -c, 0], [c, -s, 0], [0, 0, 0.0]])
        centre = np.array([p[0], p[1], p[3] if len(p) > 3 else z])
        d = L @ R.T + centre - self.t_wc
        n = np.linalg.norm(d, axis=1, keepdims=True)
        q = d / n
        r = self.f * np.einsum("ikj,ij->ik", B, q).ravel()
        # dq/dd = (I - q q^T) / |d|; B q only needs B (I - q q^T) = B - (B q) q^T
        Bq = np.einsum("ikj,ij->ik", B, q)
        G = (B - Bq[:, :, None] * q[:, None, :]) / n[:, :, None] * self.f          # (m, 2, 3)
        cols = [G[:, :, 0], G[:, :, 1], np.einsum("ikj,ij->ik", G, L @ dR.T)]
        if len(p) > 3:
            cols.append(G[:, :, 2])
        J = np.stack([col.ravel() for col in cols], axis=1)
        return r, J

    def solve(self, p0, faces, z=None, iterations=10):
        """Gauss-Newton with Huber weights (IRLS) -> (p, residuals, covariance of p)."""
        L = np.concatenate([fc.local for fc in faces])
        B = np.concatenate([fc.basis for fc in faces])
        sw2 = np.repeat([fc.weight for fc in faces], 8)        # per-face weight on its 8 residuals
        p = np.array(p0, float)
        tol = np.array([2e-5, 2e-5, 2e-4, 2e-5])[:len(p)]   # 0.02 mm, 0.01 deg
        for _ in range(iterations):
            r, J = self._model(p, L, B, z)
            a = np.abs(r)
            w = sw2 * np.where(a <= self.huber, 1.0, self.huber / np.maximum(a, 1e-12))
            JtW = J.T * w
            delta = -np.linalg.solve(JtW @ J + 1e-9 * np.eye(len(p)), JtW @ r)
            p = p + delta
            if np.all(np.abs(delta) < tol):
                break
        r, J = self._model(p, L, B, z)
        dof = max(len(r) - len(p), 1)
        sigma2 = max(float(np.sum(sw2 * r ** 2) / max(np.sum(sw2), 1e-12) * len(r) / dof), 0.25 ** 2)
        try:
            cov = np.linalg.inv((J.T * sw2) @ J) * sigma2
        except np.linalg.LinAlgError:
            cov = np.full((len(p), len(p)), np.inf)
        return p, r, cov


@dataclass
class FusionParams:
    """Tunables of cube_estimates(); the defaults come from the Monte Carlo in test/."""
    max_reproj_px: float = 3.0      # IPPE corner error above this: the marker is not used
    min_view_cos: float = 0.2       # faces seen more grazing than this are not used
    snap_window: float = 0.02       # m; table prior only if the faces put the centre this close to it
    max_tilt_deg: float = 20.0      # a face pose more tilted than this proposes no upright yaw
    assign_deg: float = 25.0        # a face must be this close to a face of the upright cube
    huber_px: float = 1.0           # residuals beyond this count linearly (robust loss)
    outlier_px: float = 2.0         # a face whose own RMS residual exceeds this (and 3x the median) is dropped
    outlier_mm: float = 5.0         # 3+ faces: a face whose centre is this far from the others' median is dropped
    yaw_mode: str = "blend"         # fused: yaw of the joint fit; top: the top face's alone when seen;
    #                                 blend: mean of the face-implied yaws, the top face weighted top_weight
    top_weight: float = 2.0
    top_min_view_cos: float = 0.3   # the top face counts as seen well enough from this view cosine
    top_min_side_px: float = 10.0   # ... and this marker size


def _candidates(fc, yaw, max_angle, t_wc):
    """Cube faces (S: marker -> cube frame) the marker can be on, for an upright cube at this yaw,
    from either IPPE solution -> [(angle, S, solution index)], best first, one per cube face."""
    Rc = _rz(yaw)
    found = {}
    for j, (R_w, t_w, _) in enumerate(fc.solutions):
        M = Rc.T @ R_w
        traces = np.einsum("nij,ij->n", _CUBE_ROT_ARRAY, M)       # trace(S^T M) for all 24 S
        i = int(np.argmax(traces))
        a = float(np.arccos(np.clip((traces[i] - 1) / 2, -1, 1)))
        S = CUBE_ROTATIONS[i]
        normal = S[:, 2]
        if a > max_angle or normal[2] < -0.5:            # the bottom face cannot be seen
            continue
        if (Rc @ normal) @ (t_wc - t_w) <= 0:             # a face turned away from the camera
            continue
        key = tuple(np.round(normal).astype(int))
        if key not in found or a < found[key][0]:
            found[key] = (a, S, j)
    return sorted(found.values(), key=lambda x: x[0])


def _with_assignment(fc, S, j):
    g = fc.pose.geometry
    local = (g.marker_obj + np.array([0, 0, g.edge / 2])) @ S.T
    return _Face(fc.pose, fc.bearings, fc.basis, fc.solutions, fc.view_cos, fc.weight, S, local, j)


def _assignments(items, yaw, max_angle, t_wc, limit=64):
    """Every way to put the markers on distinct faces of an upright cube at this yaw.
    -> lists of assigned _Face (markers with no possible face are left out). When no assignment
    keeps all of them on distinct faces, subsets are tried as well."""
    cands = [[_with_assignment(fc, S, j) for _, S, j in _candidates(fc, yaw, max_angle, t_wc)] for fc in items]
    cands = [c for c in cands if c]

    def distinct(faces):
        normals = [tuple(np.round(f.S[:, 2]).astype(int)) for f in faces]
        return len(set(normals)) == len(normals)

    full = [list(c) for c in itertools.islice(itertools.product(*cands), limit) if distinct(c)]
    if full:
        return full
    out = []
    for c in itertools.islice(itertools.product(*[cs + [None] for cs in cands]), limit):
        faces = [f for f in c if f is not None]
        if faces and distinct(faces):
            out.append(faces)
    return out


def _initial_centre(faces, yaw, z_fixed, t_wc):
    """Weighted median of the face-implied centres (each slid along its ray to z_fixed if given)."""
    Rc = _rz(yaw)
    pts, ws = [], []
    for fc in faces:
        _, t_w, _ = fc.solutions[fc.solution]
        offset = -Rc @ fc.S[:, 2] * fc.pose.geometry.edge / 2
        c = t_w + offset
        if z_fixed is not None:
            ray = t_w - t_wc
            if abs(ray[2]) > 1e-6:
                s = (z_fixed - offset[2] - t_wc[2]) / ray[2]
                if 0.5 < s < 2.0:
                    c = t_wc + s * ray + offset
        pts.append(c)
        ws.append(fc.weight)
    pts, ws = np.array(pts), np.array(ws)
    return np.array([_weighted_median(pts[:, i], ws) for i in range(3)])


def _weighted_median(x, w):
    i = np.argsort(x)
    c = np.cumsum(w[i])
    return float(x[i][np.searchsorted(c, c[-1] / 2)])


def _yaw_hypotheses(faces, max_tilt, cluster=np.radians(8), keep=3):
    """Yaw (mod 90) proposals from every face's IPPE solutions that stand upright, best supported first."""
    cands = []
    for fc in faces:
        for j, (R_w, _, err) in enumerate(fc.solutions):
            up = int(np.argmax(np.abs(R_w[2])))
            tilt = np.arccos(min(1.0, abs(R_w[2, up])))
            if tilt > max_tilt:
                continue
            horiz = R_w[:, (up + 1) % 3]
            cands.append((float(fold_yaw(np.arctan2(horiz[1], horiz[0]))), fc.weight / (1.0 + err) / (1 + j)))
    if not cands:
        return []
    ang = np.array([c[0] for c in cands])
    w = np.array([c[1] for c in cands])
    hyps = []
    for _ in range(keep):
        if w.sum() <= 0:
            break
        near = np.abs(fold_yaw(ang[:, None] - ang[None, :])) < cluster
        i = int(np.argmax(near @ w))
        members = near[i] & (w > 0)
        hyps.append(circular_mean_90(ang[members], w[members])[0])
        w = np.where(members, 0.0, w)
    return hyps


def _huber_cost(r, delta):
    a = np.abs(r)
    return float(np.sum(np.where(a <= delta, a ** 2 / 2, delta * (a - delta / 2))))


def _legacy_average(k, items, R_wc, t_wc):
    """A cube that does not stand upright (tipped over, held): weighted mean of the face poses."""
    centres, weights, yaws, tilts = [], [], [], []
    for fc in items:
        p = fc.pose
        axes = R_wc @ p.R
        up = int(np.argmax(np.abs(axes[2])))
        tilts.append(np.arccos(min(1.0, abs(axes[2, up]))))
        horiz = axes[:, (up + 1) % 3]
        yaws.append(np.arctan2(horiz[1], horiz[0]))
        centres.append(R_wc @ p.cube_centre + t_wc)
        weights.append(fc.weight)
    yaw, _ = circular_mean_90(yaws, weights)
    return CubeEstimate(k, np.average(centres, axis=0, weights=weights), yaw, float(np.max(tilts)),
                        [fc.pose.marker_id for fc in items], float(np.mean([fc.pose.reproj_px for fc in items])),
                        confidence=0.1, side_px=float(np.mean([fc.pose.side_px for fc in items])))


def cube_estimates(poses, R_wc, t_wc, max_reproj_px=None, min_view_cos=None, table_centre_z=None,
                   snap_window=None, table_z=None, params=None, **overrides):
    """Marker poses in the camera frame + camera pose in the target frame (R_wc, t_wc) -> one
    CubeEstimate per cube from its visible faces (method: module docstring; tunables: FusionParams,
    given as `params` and/or keyword overrides).

    Table prior: `table_z` (the table surface; a resting cube's centre is half its own edge above)
    or `table_centre_z` (that centre height directly, for cubes of the nominal edge). It is used
    when the faces put the cube within `snap_window` of it (a stacked or held cube is not).

    Faces are weighted by view cosine (a grazing face's corners are less precise) and by the
    marker's size in the image (up to 20 px).

    confidence (0..1) = faces term (1 - 0.5^n: one face 0.5, two 0.75, three 0.88)
                        x residual term exp(-rms^2 / 2) (rms of the fused fit, px)
                        x size term min(1, marker side / 20 px)
                        x consistency term exp(-(face spread / 3 mm)^2 / 2)
                        x 0.75 without a well seen top face (side faces give the yaw less well)
    A cube that does not stand upright gets the plain average of its face poses and confidence 0.1.
    """
    prm = params or FusionParams()
    given = {"max_reproj_px": max_reproj_px, "min_view_cos": min_view_cos, "snap_window": snap_window}
    overrides.update({k: v for k, v in given.items() if v is not None})
    if overrides:
        prm = FusionParams(**{**prm.__dict__, **overrides})
    by_cube = {}
    for p in poses:
        if p.reproj_px > prm.max_reproj_px or p.bearings is None:
            continue
        view_cos = float(-(p.R[:, 2] @ p.t) / np.linalg.norm(p.t))
        if view_cos < prm.min_view_cos:
            continue
        b_w = p.bearings @ R_wc.T
        sols = [(R_wc @ R, R_wc @ t + t_wc, e) for R, t, e in (p.solutions or [(p.R, p.t, p.reproj_px)])]
        weight = view_cos * min(1.0, p.side_px / 20.0)
        by_cube.setdefault(p.cube, []).append(_Face(p, b_w, _ray_basis(b_w), sols, view_cos, weight))
    out = []
    for k, items in sorted(by_cube.items()):
        g = items[0].pose.geometry
        zc = table_z + g.edge / 2 if table_z is not None else table_centre_z
        out.append(_fit_cube(k, items, R_wc, t_wc, zc, prm))
    return out


def _fit_cube(k, items, R_wc, t_wc, zc, prm):
    hyps = _yaw_hypotheses(items, np.radians(prm.max_tilt_deg))
    if not hyps:
        return _legacy_average(k, items, R_wc, t_wc)
    fit = _Fit(t_wc, items[0].pose.f_px or 1.0, prm.huber_px)

    # yaw hypothesis x marker-to-face assignment with the lowest robust cost; markers left out cost extra
    best = None
    for yaw0 in hyps:
        for faces in _assignments(items, yaw0, np.radians(prm.assign_deg), t_wc):
            if not faces:
                continue
            free = _initial_centre(faces, yaw0, None, t_wc)
            z = zc if zc is not None and abs(free[2] - zc) < prm.snap_window else None
            c0 = _initial_centre(faces, yaw0, z, t_wc)
            p0 = [c0[0], c0[1], yaw0] + ([] if z is not None else [c0[2]])
            p, r, _ = fit.solve(p0, faces, z, iterations=4)
            cost = _huber_cost(r, prm.huber_px) + (len(items) - len(faces)) * 8 * prm.outlier_px ** 2
            if best is None or cost < best[0]:
                best = (cost, faces, z, p)
    if best is None:
        return _legacy_average(k, items, R_wc, t_wc)
    _, faces, z, p = best
    assigned = {id(fc.pose) for fc in faces}
    rejected = [fc.pose.marker_id for fc in items if id(fc.pose) not in assigned]

    # drop faces that do not fit the others, worst first
    while True:
        p, r, cov = fit.solve(p, faces, z)
        if len(faces) < 2:
            break
        rms = np.sqrt((r.reshape(len(faces), 8) ** 2).mean(axis=1))
        if len(faces) >= 3:
            singles = np.array([fit.solve(p, [fc], z, iterations=6)[0][:2] for fc in faces])
            dist = np.linalg.norm(singles - np.median(singles, axis=0), axis=1)
        else:
            dist = np.zeros(len(faces))
        bad = (rms > max(prm.outlier_px, 3 * np.median(rms))) | (dist > prm.outlier_mm / 1000)
        if not bad.any():
            break
        i = int(np.argmax(np.where(bad, rms / prm.outlier_px + dist * 1000 / prm.outlier_mm, -1)))
        rejected.append(faces[i].pose.marker_id)
        faces = faces[:i] + faces[i + 1:]

    centre = np.array([p[0], p[1], p[3] if z is None else z])
    rms_all = float(np.sqrt(np.mean(r ** 2)))

    # what every face implies alone, and which one is the top face
    face_centres, face_yaws, face_w, top, top_q = {}, {}, {}, -1, 0.0
    for fc in faces:
        s = fit.solve(p, [fc], z, iterations=6)[0] if len(faces) > 1 else p
        mid = fc.pose.marker_id
        face_centres[mid] = np.array([s[0], s[1], s[3] if z is None else z])
        face_yaws[mid] = float(fold_yaw(s[2]))
        face_w[mid] = fc.weight
        if fc.S[2, 2] > 0.5 and fc.view_cos >= prm.top_min_view_cos and fc.pose.side_px >= prm.top_min_side_px:
            if fc.weight > top_q:
                top, top_q = mid, fc.weight
    yaw = float(fold_yaw(p[2]))
    if len(faces) > 1 and prm.yaw_mode == "top" and top >= 0:
        yaw = face_yaws[top]
    elif len(faces) > 1 and prm.yaw_mode == "blend":
        ids = list(face_yaws)
        w = [face_w[i] * (prm.top_weight if i == top else 1.0) for i in ids]
        yaw = circular_mean_90([face_yaws[i] for i in ids], w)[0]
    if len(faces) > 1:
        spread = float(np.sqrt(np.mean([np.sum((c[:2] - centre[:2]) ** 2) for c in face_centres.values()])))
        spread_yaw = float(np.sqrt(np.mean([fold_yaw(a - yaw) ** 2 for a in face_yaws.values()])))
    else:
        spread = spread_yaw = 0.0
    side = float(np.mean([fc.pose.side_px for fc in faces]))
    confidence = (1 - 0.5 ** len(faces)) * np.exp(-rms_all ** 2 / 2) * min(1.0, side / 20.0) \
        * np.exp(-(spread * 1000 / 3.0) ** 2 / 2) * (1.0 if top >= 0 else 0.75)
    return CubeEstimate(
        k, centre, yaw, 0.0, [fc.pose.marker_id for fc in faces], rms_all,
        confidence=float(confidence), sigma_xy=float(np.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))),
        sigma_yaw=float(np.sqrt(max(cov[2, 2], 0.0))), face_spread=spread, face_spread_yaw=spread_yaw,
        top_face=top, rejected=rejected, on_table=z is not None, side_px=side,
        face_centres=face_centres, face_yaws=face_yaws)


def estimate_cubes(detections, K, D, R_wc, t_wc, geometry=None, **kwargs):
    """Marker detections (id, 4x2 px) of one image -> CubeEstimate per cube; kwargs: cube_estimates()."""
    return cube_estimates(marker_poses(detections, K, D, geometry), R_wc, t_wc, **kwargs)


def scaled_camera(K, image_width, calib_width=1280):
    """K of the calibration image scaled to a published image of another size (D is unchanged:
    the equidistant model works on normalised coordinates)."""
    s = image_width / calib_width
    K = K.copy()
    K[0, 0] *= s
    K[1, 1] *= s
    K[0, 2] *= s
    K[1, 2] *= s
    return K
