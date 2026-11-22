"""Synthetic wrist-camera images of imperfect paper cubes, for the cube detector's tests.

A cube is ray-traced through the wrist camera's fisheye model. Every face can be imperfect the way
a folded paper cube is:

- the face plane is tilted against the ideal cube face and shifted along its normal,
- the face bulges (or dents) towards its centre,
- the printed marker is rotated in the face, off-centre and scaled,
- the whole cube is a little larger or smaller than nominal and stands slightly tilted,
- every face has its own brightness (lighting).

The image is supersampled, blurred, noised and JPEG-compressed like the published camera stream.
No ROS; only numpy and OpenCV.
"""

import os
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import cube_detection as cd  # noqa: E402

CALIB = os.path.join(HERE, "..", "config", "wrist_camera_fisheye.yaml")
W, H = 640, 360   # the published size (scale 0.5)
EDGE = 0.030      # nominal cube edge and marker size (what the detector assumes)
MARKER = 0.022

BLACK, WHITE = 25, 230


def camera(width=W):
    with open(CALIB) as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
    return cd.scaled_camera(K, width, c["image_width"]), D


def rot(axis, angle):
    axis = np.asarray(axis, float)
    return cv2.Rodrigues(axis / np.linalg.norm(axis) * angle)[0]


def marker_texture(marker_id, cells=6, px_per_cell=20):
    """Marker (cells x cells including its black border) with one white cell around it (quiet zone)."""
    side = cells * px_per_cell
    d = cv2.aruco.getPredefinedDictionary(cd.DICTIONARY)
    img = cv2.aruco.generateImageMarker(d, marker_id, side) if hasattr(cv2.aruco, "generateImageMarker") \
        else cv2.aruco.drawMarker(d, marker_id, side)
    return cv2.copyMakeBorder(img, px_per_cell, px_per_cell, px_per_cell, px_per_cell, cv2.BORDER_CONSTANT, value=255)


def ideal_face_frames():
    """Rotation (face -> cube) of the six faces, z out of the face: +z, -z, +x, -x, +y, -y."""
    faces = []
    for axis, sign in [(2, 1), (2, -1), (0, 1), (0, -1), (1, 1), (1, -1)]:
        z = np.zeros(3)
        z[axis] = sign
        x = np.array([1.0, 0, 0]) if axis != 0 else np.array([0, 1.0, 0])
        faces.append(np.c_[x, np.cross(z, x), z])
    return faces


@dataclass
class Face:
    R: np.ndarray                 # face -> cube rotation (tilted), z out of the face
    offset_n: float = 0.0         # face plane shift along its normal, m
    bulge: float = 0.0            # outward displacement at the face centre, m (negative: dent)
    marker_rot: float = 0.0       # in-plane rotation of the printed marker, rad
    marker_offset: np.ndarray = field(default_factory=lambda: np.zeros(2))   # m, in the face plane
    marker_scale: float = 1.0
    shade: float = 1.0


@dataclass
class PaperCube:
    k: int
    centre: np.ndarray            # world, m
    yaw: float                    # rad
    edge: float = EDGE
    marker: float = MARKER
    tilt: np.ndarray = field(default_factory=lambda: np.eye(3))   # small whole-cube tilt (wobbly bottom)
    faces: list = field(default_factory=list)
    ids: list = field(default_factory=list)   # marker id per face

    @property
    def R(self):
        return rot([0, 0, 1], self.yaw) @ self.tilt


def ideal_cube(k, centre, yaw, edge=EDGE, marker=MARKER, ids=None):
    return PaperCube(k, np.asarray(centre, float), yaw, edge, marker, np.eye(3),
                     [Face(R) for R in ideal_face_frames()], list(ids or range(6 * k, 6 * k + 6)))


@dataclass
class Imperfections:
    """Upper bounds of the uniform random perturbations (a paper cube folded by hand)."""
    face_tilt_deg: float = 4.0
    face_shift_mm: float = 0.5
    bulge_mm: float = 1.5
    marker_rot_deg: float = 3.0
    marker_offset_mm: float = 2.0
    marker_scale: float = 0.02
    edge_scale: float = 0.02
    cube_tilt_deg: float = 2.0
    shade: tuple = (0.55, 1.0)

    @classmethod
    def none(cls):
        return cls(0, 0, 0, 0, 0, 0, 0, 0, (1.0, 1.0))


def paper_cube(k, centre_xy, yaw, table_z, rng, imp=Imperfections()):
    """A randomly imperfect cube k resting on the table at centre_xy, yaw."""
    edge = EDGE * (1 + rng.uniform(-imp.edge_scale, imp.edge_scale))
    tilt_axis = np.r_[rng.normal(size=2), 0.0]
    tilt = rot(tilt_axis, np.radians(rng.uniform(0, imp.cube_tilt_deg))) if imp.cube_tilt_deg > 0 else np.eye(3)
    faces = []
    for R in ideal_face_frames():
        a = rng.uniform(0, 2 * np.pi)
        R_t = rot([np.cos(a), np.sin(a), 0], np.radians(rng.uniform(0, imp.face_tilt_deg))) \
            if imp.face_tilt_deg > 0 else np.eye(3)
        faces.append(Face(
            R=R @ R_t,
            offset_n=rng.uniform(-1, 1) * imp.face_shift_mm / 1000,
            bulge=rng.uniform(-0.3, 1.0) * imp.bulge_mm / 1000,
            marker_rot=np.radians(rng.uniform(-1, 1) * imp.marker_rot_deg),
            marker_offset=rng.uniform(-1, 1, 2) * imp.marker_offset_mm / 1000,
            marker_scale=1 + rng.uniform(-1, 1) * imp.marker_scale,
            shade=rng.uniform(*imp.shade)))
    ids = list(range(6 * k, 6 * k + 6))
    rng.shuffle(ids)                       # which id is on which face does not matter
    centre = np.array([centre_xy[0], centre_xy[1], table_z + edge / 2])
    return PaperCube(k, centre, yaw, edge, MARKER, tilt, faces, ids)


def look_at(eye, target, roll=0.0):
    """Camera (optical frame: z forward, y down) at eye looking at target, then rolled about z."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, [0, 0, 1.0])
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0, -1.0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.c_[x, y, z] @ rot([0, 0, 1], roll), eye


def _trace(cubes, rays_w, t_wc, table_z, table_grey):
    """Grey value of every world ray (n x 3)."""
    n = len(rays_w)
    val = np.full(n, float(table_grey))
    # the table plane (z = table_z) where the ray goes down
    depth = np.full(n, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        s_tab = (table_z - t_wc[2]) / rays_w[:, 2]
    depth[(s_tab > 0)] = s_tab[s_tab > 0]
    for cube in cubes:
        Rc = cube.R
        h = cube.edge / 2
        for f, face in enumerate(cube.faces):
            Rf = Rc @ face.R                       # face -> world
            nrm = Rf[:, 2]
            p0 = cube.centre + (h + face.offset_n) * nrm
            denom = rays_w @ nrm
            front = denom < -1e-9
            if not front.any():
                continue
            disp = np.zeros(n)
            for _ in range(3):                     # bulged surface: fixed-point on the displacement
                with np.errstate(divide="ignore", invalid="ignore"):
                    s = ((p0 + disp[:, None] * nrm - t_wc) @ nrm) / denom
                local = (t_wc + s[:, None] * rays_w - p0) @ Rf
                a, b = local[:, 0], local[:, 1]
                ua, ub = np.clip(np.abs(a) / h, 0, 1), np.clip(np.abs(b) / h, 0, 1)
                disp = face.bulge * (1 - ua ** 2) * (1 - ub ** 2)
            hit = front & (s > 0) & (s < depth) & (np.abs(a) <= h) & (np.abs(b) <= h)
            if not hit.any():
                continue
            # marker coordinates: undo its offset, rotation and scale on the face
            c, sn = np.cos(face.marker_rot), np.sin(face.marker_rot)
            da, db = a[hit] - face.marker_offset[0], b[hit] - face.marker_offset[1]
            ma = (c * da + sn * db) / face.marker_scale
            mb = (-sn * da + c * db) / face.marker_scale
            tex = marker_texture(cube.ids[f])
            span = cube.marker * 8 / 6
            tu = (ma / span + 0.5) * tex.shape[1]
            tv = (-mb / span + 0.5) * tex.shape[0]
            inside = (tu >= 0) & (tu < tex.shape[1]) & (tv >= 0) & (tv < tex.shape[0])
            v = np.full(hit.sum(), 255.0)
            v[inside] = tex[tv[inside].astype(int), tu[inside].astype(int)]
            v = (BLACK + (WHITE - BLACK) * v / 255.0) * face.shade
            val[hit] = v
            depth[hit] = s[hit]
    return val


def render(cubes, R_wc, t_wc, K, D, table_z=0.0, rng=None, supersample=3, table_grey=120,
           blur_sigma=0.7, noise_sigma=2.0, jpeg_quality=80, width=W, height=H):
    """Grey uint8 image of the cubes on a uniform table; supersampled only around the cubes."""
    img = np.full((height, width), float(table_grey))
    # region of interest: the projected (generously sized) cube corners
    rois = []
    R_cw, t_cw = R_wc.T, -R_wc.T @ t_wc
    for cube in cubes:
        e = cube.edge * 0.6
        corners = np.array([[x, y, z] for x in (-e, e) for y in (-e, e) for z in (-e, e)]) @ cube.R.T + cube.centre
        pc = corners @ R_cw.T + t_cw
        if np.all(pc[:, 2] <= 0.01):
            continue
        pc = pc[pc[:, 2] > 0.01]
        px, _ = cv2.fisheye.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        px = px.reshape(-1, 2)
        x0, y0 = np.floor(px.min(axis=0)).astype(int) - 4
        x1, y1 = np.ceil(px.max(axis=0)).astype(int) + 4
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, width), min(y1, height)
        if x1 > x0 and y1 > y0:
            rois.append((x0, y0, x1, y1))
    for x0, y0, x1, y1 in rois:
        ss = supersample
        off = (np.arange(ss) + 0.5) / ss
        u = (np.arange(x0, x1)[:, None] + off[None, :]).ravel()
        v = (np.arange(y0, y1)[:, None] + off[None, :]).ravel()
        uu, vv = np.meshgrid(u, v)
        px = np.c_[uu.ravel(), vv.ravel()].reshape(-1, 1, 2)
        xy = cv2.fisheye.undistortPoints(px, K, D).reshape(-1, 2)
        rays_w = np.c_[xy, np.ones(len(xy))] @ R_wc.T
        val = _trace(cubes, rays_w, t_wc, table_z, table_grey).reshape(len(v), len(u))
        img[y0:y1, x0:x1] = val.reshape(y1 - y0, ss, x1 - x0, ss).mean(axis=(1, 3))
    if blur_sigma > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur_sigma)
    if rng is not None and noise_sigma > 0:
        img = img + rng.normal(0, noise_sigma, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    if jpeg_quality:
        img = cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])[1], cv2.IMREAD_GRAYSCALE)
    return img


def random_view(rng, centre):
    """A wrist-camera pose looking at a cube from where the arm can be: 12-30 cm above the table,
    from straight above (pre-grasp) to oblique (searching), the cube anywhere in the inner image."""
    height = rng.uniform(0.12, 0.30)
    back = rng.uniform(0.0, 0.30)                     # horizontal distance, towards the base mostly
    az = np.arctan2(-centre[1], -centre[0]) + rng.uniform(-1.0, 1.0)
    eye = centre + np.array([back * np.cos(az), back * np.sin(az), height])
    aim = centre + np.r_[rng.normal(0, 0.06, 2), 0.0]
    return look_at(eye, aim, roll=rng.uniform(-np.pi, np.pi))


def random_scene(rng, imp=Imperfections(), table_z=0.0, k=None):
    """One imperfect cube at a random place and yaw, seen from a random wrist pose."""
    k = int(rng.integers(0, 6)) if k is None else k
    xy = np.array([rng.uniform(0.18, 0.38), rng.uniform(-0.15, 0.15)])
    cube = paper_cube(k, xy, rng.uniform(-np.pi, np.pi), table_z, rng, imp)
    R_wc, t_wc = random_view(rng, cube.centre)
    return cube, R_wc, t_wc


def yaw_error(est, truth):
    """Folded difference of two yaw angles (cube symmetry: 90 deg), rad."""
    return float(cd.fold_yaw(est - truth))
