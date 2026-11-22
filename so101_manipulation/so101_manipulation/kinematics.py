"""SO-101 forward kinematics and table model, straight from the URDF and its meshes (no ROS).

Same model as tools/so101_hand_eye.py (FK) and tools/so101_table_touch.py (lowest point of the
arm above the table), without their bus and camera dependencies:

- so101_description/urdf/so101_new_calib.urdf, with the shoulder_lift / elbow_flex / wrist_flex
  origins taken from so101_description/config/so101_kinematics.yaml (the calibrated values the
  xacro uses as well), and table_z from the same file;
- the wrist camera links from wrist_camera.xacro, so the camera counts in the clearance check;
- every checked link is reduced to the convex hull of its visual meshes: the lowest point of a
  rigid body above a plane is always a hull vertex.

Angles are radians unless a name says _deg; lengths are metres unless _mm.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
BASE = "base_link"
HAND = "gripper_link"
TIP = "gripper_frame_link"            # the FIXED jaw tip, not the middle of the jaw gap
MOVING_JAW = "moving_jaw_so101_v1_link"

# MoveIt planning limits (so101_moveit_config/config/joint_limits.yaml), radians
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.700, 1.74533),
    "elbow_flex": (-1.69, 1.62),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

URDF_LINKS = ["upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", MOVING_JAW]
CAMERA_LINKS = ["wrist_camera_mount_link", "wrist_camera_link"]


def description_dir() -> Path:
    """so101_description: $SO101_DESCRIPTION_DIR, the installed share, or the source tree."""
    env = os.environ.get("SO101_DESCRIPTION_DIR")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("so101_description"))
    except Exception:  # noqa: BLE001  (not sourced, or running from the source tree)
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "so101_description"
        if (cand / "urdf" / "so101_new_calib.urdf").exists():
            return cand
    raise FileNotFoundError("so101_description not found: source the workspace or set SO101_DESCRIPTION_DIR")


def perception_config_dir() -> Path:
    env = os.environ.get("SO101_PERCEPTION_CONFIG_DIR")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("so101_perception")) / "config"
    except Exception:  # noqa: BLE001
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "so101_perception" / "config"
        if (cand / "wrist_camera_hand_eye.yaml").exists():
            return cand
    raise FileNotFoundError("so101_perception/config not found: source the workspace or set SO101_PERCEPTION_CONFIG_DIR")


# --- small rigid-body helpers -------------------------------------------------------------

def rpy_matrix(r, p, y):
    """URDF convention: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


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


def axis_angle(axis, angle):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def origin_transform(element):
    o = element.find("origin")
    if o is None:
        return np.eye(4)
    return transform(rpy_matrix(*map(float, o.get("rpy", "0 0 0").split())),
                     [float(v) for v in o.get("xyz", "0 0 0").split()])


def stl_vertices(path: Path) -> np.ndarray:
    """Vertices of a binary STL."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    return tri["v"].reshape(-1, 3).astype(np.float64)


# --- chains ------------------------------------------------------------------------------

class Chain:
    """Joints from a base link to a tool link: [(name, origin 4x4, axis or None)]."""

    def __init__(self, joints):
        self.joints = joints

    def __add__(self, other):
        return Chain(self.joints + other.joints)

    def forward(self, q: dict) -> np.ndarray:
        T = np.eye(4)
        for name, origin, axis in self.joints:
            T = T @ origin
            if axis is not None:
                a = q.get(name, 0.0)
                if a:
                    R = np.eye(4)
                    R[:3, :3] = axis_angle(axis, a)
                    T = T @ R
        return T


def load_chain(path: Path, base: str, tool: str, kinematics: dict | None = None) -> Chain:
    kinematics = kinematics or {}
    root = ET.parse(path).getroot()
    joints = [j for j in root.iter("joint") if j.find("child") is not None]
    by_child = {j.find("child").get("link"): j for j in joints}
    chain, link = [], tool
    while link != base:
        j = by_child[link]
        axis = None if j.get("type") == "fixed" else np.array([float(v) for v in j.find("axis").get("xyz").split()])
        if axis is not None:
            axis = axis / np.linalg.norm(axis)
        if j.get("name") in kinematics:
            o = kinematics[j.get("name")]
            origin = transform(rpy_matrix(*o["rpy"]), o["xyz"])
        else:
            origin = origin_transform(j)
        chain.append((j.get("name"), origin, axis))
        link = j.find("parent").get("link")
    return Chain(chain[::-1])


def link_hull(path: Path, link: str, mesh_dir: Path) -> np.ndarray:
    from scipy.spatial import ConvexHull
    for el in ET.parse(path).getroot().iter("link"):
        if el.get("name") != link:
            continue
        pts = []
        for v in el.findall("visual"):
            mesh = v.find("geometry/mesh")
            scale = float((mesh.get("scale") or "1").split()[0])
            T = origin_transform(v)
            pts.append(stl_vertices(mesh_dir / Path(mesh.get("filename")).name) * scale @ T[:3, :3].T + T[:3, 3])
        P = np.vstack(pts)
        return P[ConvexHull(P).vertices]
    raise KeyError(link)


def link_vertices(path: Path, link: str, mesh_dir: Path) -> np.ndarray:
    for el in ET.parse(path).getroot().iter("link"):
        if el.get("name") != link:
            continue
        pts = []
        for v in el.findall("visual"):
            mesh = v.find("geometry/mesh")
            scale = float((mesh.get("scale") or "1").split()[0])
            T = origin_transform(v)
            pts.append(stl_vertices(mesh_dir / Path(mesh.get("filename")).name) * scale @ T[:3, :3].T + T[:3, 3])
        return np.vstack(pts)
    raise KeyError(link)


class ArmModel:
    """FK of the hand, the jaw tips, the camera, and the lowest point of the arm above the table."""

    def __init__(self, desc_dir: Path | None = None, table_z: float | None = None):
        d = Path(desc_dir) if desc_dir is not None else description_dir()
        self.urdf = d / "urdf" / "so101_new_calib.urdf"
        self.camera_xacro = d / "urdf" / "wrist_camera.xacro"
        self.mesh_dir = d / "meshes"
        with open(d / "config" / "so101_kinematics.yaml") as f:
            kin = yaml.safe_load(f)
        self.kinematics = kin["joints"]
        self.table_z = float(kin.get("table_z", 0.0)) if table_z is None or not np.isfinite(table_z) else float(table_z)

        self.hand_chain = load_chain(self.urdf, BASE, HAND, self.kinematics)
        # gripper_link -> fixed tip, and gripper_link -> moving jaw (joint "gripper")
        self.T_hand_tip = load_chain(self.urdf, HAND, TIP).forward({})
        self.jaw_chain = load_chain(self.urdf, HAND, MOVING_JAW)
        # moving jaw tip: the jaw mesh vertex farthest from the jaw joint
        P = link_vertices(self.urdf, MOVING_JAW, self.mesh_dir)
        self.moving_tip_local = P[np.argmax(np.linalg.norm(P, axis=1))]
        self.links = [(name, load_chain(self.urdf, BASE, name, self.kinematics), link_hull(self.urdf, name, self.mesh_dir))
                      for name in URDF_LINKS]
        cam_parent = load_chain(self.urdf, BASE, HAND, self.kinematics)
        self.links += [(name, cam_parent + load_chain(self.camera_xacro, "${parent}", name),
                        link_hull(self.camera_xacro, name, self.mesh_dir)) for name in CAMERA_LINKS]
        # closing direction of the moving jaw tip in gripper_link (unit), as in grasp_ik.py
        a = self.jaw_point({GRIPPER: 0.3})
        b = self.jaw_point({GRIPPER: 0.0})
        self.close_dir_local = (b - a) / np.linalg.norm(b - a)

    # q: dict joint -> rad; arm vectors: [pan, lift, elbow, wflex, roll]
    @staticmethod
    def qdict(q_arm, grip=0.0) -> dict:
        d = dict(zip(ARM, map(float, q_arm)))
        d[GRIPPER] = float(grip)
        return d

    def hand(self, q_arm) -> np.ndarray:
        return self.hand_chain.forward(dict(zip(ARM, q_arm)))

    def jaw_point(self, q_grip: dict) -> np.ndarray:
        """moving jaw tip in gripper_link."""
        T = self.jaw_chain.forward(q_grip)
        return T[:3, :3] @ self.moving_tip_local + T[:3, 3]

    def tips(self, q_arm, grip):
        """(T_hand, fixed tip xyz, moving tip xyz) in base_link."""
        Th = self.hand(q_arm)
        fixed = Th[:3, :3] @ self.T_hand_tip[:3, 3] + Th[:3, 3]
        moving = Th[:3, :3] @ self.jaw_point({GRIPPER: grip}) + Th[:3, 3]
        return Th, fixed, moving

    def tip(self, q_arm) -> np.ndarray:
        Th = self.hand(q_arm)
        return Th[:3, :3] @ self.T_hand_tip[:3, 3] + Th[:3, 3]

    def gap_offset(self, q_arm, grip, fraction=0.5) -> np.ndarray:
        """(point `fraction` of the way from the fixed to the moving jaw tip) - fixed tip, base_link."""
        _, fixed, moving = self.tips(q_arm, grip)
        return fraction * (moving - fixed)

    def lowest(self, q_arm, grip):
        """(link, height above the table [m], point) of the lowest point of the arm."""
        q = self.qdict(q_arm, grip)
        best = None
        for name, chain, P in self.links:
            T = chain.forward(q)
            z = P @ T[2, :3] + T[2, 3]
            i = int(np.argmin(z))
            if best is None or z[i] < best[1]:
                best = (name, float(z[i]), T[:3, :3] @ P[i] + T[:3, 3])
        return best[0], best[1] - self.table_z, best[2]

    def lowest_with_load(self, q_arm, grip, load_below=0.0) -> float:
        """Height [m] above the table of the lowest point of the arm, or of a carried object that
        hangs `load_below` metres under the fixed jaw tip (0: nothing carried)."""
        h = self.lowest(q_arm, grip)[1]
        if load_below > 0:
            h = min(h, self.tip(q_arm)[2] - load_below - self.table_z)
        return h

    def path_clearance(self, a, b, grip, load_below=0.0, n=20) -> float:
        """Smallest height above the table [m] along a straight joint-space move a -> b."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        steps = max(2, int(n))
        return min(self.lowest_with_load(a + s * (b - a), grip, load_below) for s in np.linspace(0, 1, steps))


def within_limits(q_arm, roll_limit=math.pi / 2, margin=1e-6) -> str | None:
    """None if the arm vector is inside the planning limits and |wrist_roll| <= roll_limit, else why not."""
    for name, v in zip(ARM, q_arm):
        lo, hi = JOINT_LIMITS[name]
        if not lo - margin <= v <= hi + margin:
            return f"{name} {math.degrees(v):.1f} deg outside [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]"
    if abs(q_arm[4]) > roll_limit + margin:
        return f"wrist_roll {math.degrees(q_arm[4]):.1f} deg beyond +-{math.degrees(roll_limit):.0f} deg (camera cable)"
    return None
