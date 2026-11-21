"""ArUco cube detection without ROS: fisheye image -> marker poses -> cube centres and yaw.

The cubes: edge 30 mm, one DICT_4X4_50 marker per face, black square 22 mm, cube k carries
ids 6k .. 6k+5 (k = 0..5). Which id sits on which face does not matter here: every face's axes
are the cube's axes (up to order and sign), so one visible face gives the centre (15 mm behind
the face) and the cube's rotation about the vertical (modulo 90 deg).

Pose convention of OpenCV ArUco: marker x right, y up, z out of the face towards the viewer.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

CUBE_EDGE = 0.030
MARKER_SIZE = 0.022
FACES_PER_CUBE = 6
N_CUBES = 6
DICTIONARY = cv2.aruco.DICT_4X4_50

# marker corners in the marker frame, ArUco order: top-left, top-right, bottom-right, bottom-left
_h = MARKER_SIZE / 2
MARKER_OBJ = np.array([[-_h, _h, 0], [_h, _h, 0], [_h, -_h, 0], [-_h, -_h, 0]], dtype=np.float64)


def cube_of(marker_id):
    """Cube index of a marker id, or None for ids that belong to no cube."""
    k = marker_id // FACES_PER_CUBE
    return k if 0 <= k < N_CUBES else None


class MarkerDetector:
    """cv2.aruco across the 4.6 (functions) and 4.7+ (ArucoDetector) APIs."""

    def __init__(self):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(DICTIONARY)
        # 4.6 also exposes DetectorParameters(), but constructing it segfaults there
        params = aruco.DetectorParameters_create() if hasattr(aruco, "DetectorParameters_create") else aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 3   # small markers: a wide window slides onto the next cell
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
        return [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)]


@dataclass
class MarkerPose:
    marker_id: int
    cube: int
    R: np.ndarray            # marker -> camera rotation
    t: np.ndarray            # marker centre in the camera frame, m
    reproj_px: float         # mean corner reprojection error (pixels of the input image)
    corners: np.ndarray      # 4x2 pixels, as detected

    @property
    def cube_centre(self):
        """Cube centre in the camera frame: half an edge behind the face."""
        return self.t + self.R @ np.array([0.0, 0.0, -CUBE_EDGE / 2])


def marker_poses(detections, K, D):
    """Fisheye (equidistant) K, D of the image the corners come from -> MarkerPose per marker.

    The corners are undistorted to normalised coordinates first, then solved with IPPE_SQUARE
    on an identity camera: solvePnP has no fisheye model.
    """
    out = []
    for marker_id, px in detections:
        k = cube_of(marker_id)
        if k is None:
            continue
        norm = cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2).astype(np.float64), K, D).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(MARKER_OBJ, norm, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or tvec[2, 0] <= 0:
            continue
        proj, _ = cv2.fisheye.projectPoints(MARKER_OBJ.reshape(-1, 1, 3), rvec, tvec, K, D)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - px, axis=1).mean())
        out.append(MarkerPose(marker_id, k, cv2.Rodrigues(rvec)[0], tvec.ravel(), err, px))
    return out


@dataclass
class CubeEstimate:
    cube: int
    centre: np.ndarray                 # in the target frame, m
    yaw: float                         # rad in [-pi/4, pi/4): rotation of the cube about the frame's z
    up_axis_tilt: float                # rad: how far the cube's most vertical axis is from z
    faces: list = field(default_factory=list)   # marker ids used
    reproj_px: float = 0.0

    @property
    def quaternion(self):
        """x, y, z, w of a pure yaw rotation."""
        return np.array([0.0, 0.0, np.sin(self.yaw / 2), np.cos(self.yaw / 2)])


def fold_yaw(angle):
    """A cube looks the same every 90 deg: fold into [-pi/4, pi/4)."""
    return (angle + np.pi / 4) % (np.pi / 2) - np.pi / 4


def cube_estimates(poses, R_wc, t_wc, max_reproj_px=3.0, min_view_cos=0.35, table_centre_z=None,
                   snap_window=0.02):
    """Marker poses in the camera frame + camera pose in the target frame (R_wc, t_wc) -> one
    CubeEstimate per cube from its visible faces.

    A face seen at a grazing angle gives a poor pose: faces whose normal is less than
    `min_view_cos` towards the camera are dropped, the rest are weighted by that cosine.
    The distance along the viewing ray is the weakest part of a small marker's pose (a few mm at
    25 cm, the direction is good to well under 1 mm). With `table_centre_z` (the target-frame
    height of the centre of a cube resting on the table) every face's estimate is slid along its
    camera ray to that height, when it is within `snap_window` of it (a stacked or held cube is not).
    """
    by_cube = {}
    for p in poses:
        if p.reproj_px > max_reproj_px:
            continue
        view_cos = float(-(p.R[:, 2] @ p.t) / np.linalg.norm(p.t))
        if view_cos >= min_view_cos:
            by_cube.setdefault(p.cube, []).append((p, view_cos))
    out = []
    for k, items in sorted(by_cube.items()):
        centres, weights, yaw_vec, tilts = [], [], np.zeros(2), []
        for p, view_cos in items:
            offset = p.R @ np.array([0.0, 0.0, -CUBE_EDGE / 2])
            centre = R_wc @ (p.t + offset) + t_wc
            if table_centre_z is not None and abs(centre[2] - table_centre_z) < snap_window:
                # centre(s) = R_wc (s t + offset) + t_wc; choose s for the table height
                dz = (R_wc @ p.t)[2]
                if abs(dz) > 1e-6:
                    s = (table_centre_z - (R_wc @ offset)[2] - t_wc[2]) / dz
                    if 0.8 < s < 1.25:
                        centre = R_wc @ (s * p.t + offset) + t_wc
            w = view_cos / (1.0 + p.reproj_px)
            centres.append(centre)
            weights.append(w)
            axes = R_wc @ p.R                           # columns: the cube's axes in the target frame
            up = int(np.argmax(np.abs(axes[2])))
            tilts.append(np.arccos(min(1.0, abs(axes[2, up]))))
            horiz = axes[:, (up + 1) % 3]               # either horizontal axis works modulo 90 deg
            a = 4 * np.arctan2(horiz[1], horiz[0])      # 4x angle: 90 deg symmetric mean
            yaw_vec += w * np.array([np.cos(a), np.sin(a)])
        yaw = fold_yaw(np.arctan2(yaw_vec[1], yaw_vec[0]) / 4)
        ps = [p for p, _ in items]
        out.append(CubeEstimate(k, np.average(centres, axis=0, weights=weights), float(yaw),
                                float(np.max(tilts)), [p.marker_id for p in ps],
                                float(np.mean([p.reproj_px for p in ps]))))
    return out


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
