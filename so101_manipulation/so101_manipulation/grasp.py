"""Grasp geometry for a cube with the SO-101 jaws (no ROS).

Ported from the 2026-11-21 session scripts (autograb.py, grasp_ik.py, vgrasp.py) that grasped a
toy 3/3 times on the real arm, and adapted to cubes:

- gripper_frame_link is the FIXED jaw tip. At 30 deg open the middle of the jaw gap is ~3 cm to
  the side of it, so the IK places the point `gap_fraction` of the way from the fixed to the
  moving jaw tip over the cube centre (0.5 = the gap midpoint), and the fixed tip at the grasp
  height. This replaces autograb's iterated `fixed_tip_target` by one residual.
- The jaws close along one face normal. A cube looks the same every 90 deg about the vertical,
  so there are 4 closing directions (yaw + k * 90 deg). Each needs a different wrist_roll; the
  ones beyond +-roll_limit (90 deg: the camera cable tore at -124 deg) are dropped, the rest are
  solved and the one with a good IK and the smallest |roll| wins.
- Vertical jaws reach only ~30 cm out (~6 cm tip height there, wrist_flex near its limit). Beyond
  that a tilted approach (<= max_tilt_deg) is the fallback, or the target is rejected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from .kinematics import ARM, JOINT_LIMITS, ArmModel, quaternion_matrix, transform

DOWN = np.array([0.0, 0.0, -1.0])
WEIGHT_VERTICAL = 400.0     # residual per unit of (approach - down): ~7 mm per degree
WEIGHT_TILTED = 60.0        # fallback: lets the jaws lean up to ~15 deg out of vertical reach
WEIGHT_CLOSE = 100.0        # per unit of closing-direction error
ARM_SEEDS = ([0.6, -0.3, 1.4], [1.0, -0.9, 1.6], [0.3, 0.3, 1.2], [0.2, 0.6, 1.0])


def wrap(a: float) -> float:
    """to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def fold_quarter(a: float) -> float:
    """to [-pi/4, pi/4): a cube's yaw is only defined modulo 90 deg."""
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


@dataclass
class GripperModel:
    """Jaw gap vs gripper angle, measured 2026-11-21 (<= 30 deg): gap = zero + per_deg * angle."""
    gap_zero_mm: float = 13.0
    gap_per_deg_mm: float = 1.35

    def gap_mm(self, angle_deg: float) -> float:
        return self.gap_zero_mm + self.gap_per_deg_mm * angle_deg

    def angle_for_gap_deg(self, gap_mm: float) -> float:
        return (gap_mm - self.gap_zero_mm) / self.gap_per_deg_mm


@dataclass
class IKResult:
    q: np.ndarray
    pos_err_mm: float
    tilt_deg: float
    close_err_deg: float
    ok: bool = False
    why: str = ""

    @property
    def roll(self) -> float:
        return float(self.q[4])


@dataclass
class GraspPlan:
    """Joint poses of one grasp, all with the same closing direction (no roll change on the way down)."""
    centre_xy: np.ndarray
    cube_yaw: float
    close_yaw: float
    hover_h: float
    grasp_h: float
    hover: IKResult
    descent: list = field(default_factory=list)     # IKResults from below the hover down to the grasp
    heights: list = field(default_factory=list)
    tilted: bool = False

    @property
    def grasp(self) -> IKResult:
        return self.descent[-1] if self.descent else self.hover

    @property
    def roll(self) -> float:
        return self.hover.roll

    def summary(self) -> str:
        return (f"cube ({self.centre_xy[0] * 100:.1f}, {self.centre_xy[1] * 100:.1f}) cm yaw {math.degrees(self.cube_yaw):+.0f}, "
                f"jaws close along {math.degrees(self.close_yaw):+.0f} deg, roll {math.degrees(self.roll):+.0f}, "
                f"hover {self.hover_h * 100:.1f} cm, grasp {self.grasp_h * 100:.1f} cm, "
                f"tilt {self.grasp.tilt_deg:.1f} deg{' (tilted fallback)' if self.tilted else ''}")


class GraspPlanner:
    def __init__(self, model: ArmModel, roll_limit: float = math.pi / 2, ik_tol_mm: float = 2.0,
                 vertical_tol_deg: float = 3.0, max_tilt_deg: float = 15.0, close_tol_deg: float = 3.0,
                 allow_tilt: bool = True, min_clearance: float = 0.003):
        self.m = model
        self.roll_limit = roll_limit
        self.ik_tol_mm = ik_tol_mm
        self.vertical_tol_deg = vertical_tol_deg
        self.max_tilt_deg = max_tilt_deg
        self.close_tol_deg = close_tol_deg
        self.allow_tilt = allow_tilt
        self.min_clearance = min_clearance
        self.lo = np.array([JOINT_LIMITS[n][0] for n in ARM])
        self.hi = np.array([JOINT_LIMITS[n][1] for n in ARM])
        self.lo[4], self.hi[4] = max(self.lo[4], -roll_limit), min(self.hi[4], roll_limit)
        self._roll_map = None

    # --- closing direction -----------------------------------------------------------------

    def close_dir(self, q_arm) -> np.ndarray:
        return self.m.hand(q_arm)[:3, :3] @ self.m.close_dir_local

    def close_yaw(self, q_arm) -> float:
        c = self.close_dir(q_arm)
        return math.atan2(c[1], c[0])

    def roll_map(self):
        """(offset, slope): closing yaw ~= pan + offset + slope * roll for vertical jaws."""
        if self._roll_map is None:
            r0 = self.solve(np.array([0.22, 0.0]), 0.03, None, 0.0, roll_hint=0.0, fix_roll=0.0)
            r1 = self.solve(np.array([0.22, 0.0]), 0.03, None, 0.0, roll_hint=0.3, fix_roll=0.3)
            y0, y1 = self.close_yaw(r0.q), self.close_yaw(r1.q)
            slope = 1.0 if wrap(y1 - y0) > 0 else -1.0
            self._roll_map = (wrap(y0 - r0.q[0]), slope)
        return self._roll_map

    def roll_for(self, pan: float, close_yaw: float) -> float:
        off, slope = self.roll_map()
        return wrap((close_yaw - pan - off) * slope)

    def close_options(self, cube_yaw: float):
        """The 4 closing yaws of a cube (along its face normals)."""
        return [wrap(cube_yaw + k * math.pi / 2) for k in range(4)]

    # --- IK -----------------------------------------------------------------------------

    def tilt_deg(self, q_arm) -> float:
        z = self.m.hand(q_arm)[:3, 2]
        return math.degrees(math.acos(float(np.clip(z[2], -1, 1))))   # approach = -z of gripper_link

    def solve(self, target_xy, tip_h, close_yaw, grip, fraction=0.5, vertical_weight=WEIGHT_VERTICAL,
              roll_hint=0.0, seeds=None, fix_roll=None, roll_weight=0.0) -> IKResult:
        """Joint angles with the gap point (`fraction` from the fixed to the moving jaw tip at gripper
        angle `grip`) over target_xy, the fixed tip `tip_h` above the table, the jaws as vertical as
        `vertical_weight` asks, and closing along `close_yaw` (None: free, roll pulled to roll_hint
        by `roll_weight`)."""
        m = self.m
        target_xy = np.asarray(target_xy, float)
        z_t = m.table_z + tip_h
        cdir = None if close_yaw is None else np.array([math.cos(close_yaw), math.sin(close_yaw), 0.0])
        T_tip = m.T_hand_tip[:3, 3]
        jaw = m.jaw_point({"gripper": grip})
        lo, hi = self.lo.copy(), self.hi.copy()
        if fix_roll is not None:
            lo[4], hi[4] = fix_roll - 1e-4, fix_roll + 1e-4

        def res(x):
            Th = m.hand(x)
            R, t = Th[:3, :3], Th[:3, 3]
            fixed = R @ T_tip + t
            moving = R @ jaw + t
            gap = fixed + fraction * (moving - fixed)
            r = [(gap[0] - target_xy[0]) * 1000, (gap[1] - target_xy[1]) * 1000, (fixed[2] - z_t) * 1000]
            r += list((-R[:, 2] - DOWN) * vertical_weight)
            if cdir is not None:
                c = R @ m.close_dir_local
                r += list((c[:2] - cdir[:2]) * WEIGHT_CLOSE)
            else:
                r += [(x[4] - roll_hint) * roll_weight * 100]
            return r

        pan = math.atan2(target_xy[1], target_xy[0])
        roll0 = float(np.clip(roll_hint if fix_roll is None else fix_roll, lo[4] + 1e-3, hi[4] - 1e-3))
        if seeds is None:
            seeds = [[pan, *s, roll0] for s in ARM_SEEDS]
        best = None
        pad = np.minimum(1e-3, (hi - lo) / 4)
        for s in seeds:
            x0 = np.clip(np.asarray(s, float), lo + pad, hi - pad)
            r = least_squares(res, x0, bounds=(lo, hi), x_scale=1.0, max_nfev=200)
            if best is None or r.cost < best.cost:
                best = r
        q = best.x
        f = np.asarray(best.fun)
        err = float(np.linalg.norm(f[:3]))
        tilt = self.tilt_deg(q)
        cerr = 0.0
        if cdir is not None:
            c = self.close_dir(q)
            cerr = math.degrees(math.acos(float(np.clip(c[:2] @ cdir[:2] / max(1e-9, np.linalg.norm(c[:2])), -1, 1))))
        return IKResult(q=q, pos_err_mm=err, tilt_deg=tilt, close_err_deg=cerr)

    def check(self, r: IKResult, grip, tilted=False) -> IKResult:
        max_tilt = self.max_tilt_deg if tilted else self.vertical_tol_deg
        why = []
        if r.pos_err_mm > self.ik_tol_mm:
            why.append(f"position off by {r.pos_err_mm:.1f} mm")
        if r.tilt_deg > max_tilt:
            why.append(f"jaws {r.tilt_deg:.1f} deg off vertical (max {max_tilt:.0f})")
        if r.close_err_deg > self.close_tol_deg:
            why.append(f"closing direction off by {r.close_err_deg:.1f} deg")
        if abs(r.roll) > self.roll_limit + 1e-6:
            why.append(f"wrist_roll {math.degrees(r.roll):.0f} deg beyond the limit")
        if not why:
            low = self.m.lowest(r.q, grip)[1]
            if low < self.min_clearance:
                why.append(f"lowest point {low * 1000:.1f} mm above the table")
        r.ok, r.why = not why, "; ".join(why)
        return r

    # --- whole grasps ---------------------------------------------------------------------

    def plan_vertical(self, centre_xy, close_yaws, grip, hover_heights, grasp_h, step=0.01,
                      fraction=0.5, cube_yaw=0.0, prefer_roll=None, tilted=False):
        """Best GraspPlan over the closing options, or (None, reasons)."""
        centre_xy = np.asarray(centre_xy, float)
        pan = math.atan2(centre_xy[1], centre_xy[0])
        wv = WEIGHT_TILTED if tilted else WEIGHT_VERTICAL
        reasons, plans = [], []
        for cy in close_yaws:
            hint = self.roll_for(pan, cy) if cy is not None else (0.0 if prefer_roll is None else prefer_roll)
            if cy is not None and abs(hint) > self.roll_limit + math.radians(12):
                reasons.append(f"close {math.degrees(cy):+.0f}: needs roll {math.degrees(hint):+.0f} deg")
                continue
            hint = float(np.clip(hint, -self.roll_limit, self.roll_limit))
            plan, why = None, ""
            for hh in hover_heights:
                r = self.check(self.solve(centre_xy, hh, cy, grip, fraction, wv, roll_hint=hint,
                                          roll_weight=0.05 if cy is None else 0.0), grip, tilted)
                if not r.ok:
                    why = f"hover {hh * 100:.1f} cm: {r.why}"
                    continue
                heights = list(np.arange(hh - step, grasp_h + 1e-4, -step))
                if not heights or abs(heights[-1] - grasp_h) > 1e-4:
                    heights.append(grasp_h)
                steps_, prev, bad = [], r, None
                for h in heights:
                    s = self.check(self.solve(centre_xy, h, cy, grip, fraction, wv, roll_hint=prev.roll,
                                              seeds=[prev.q], roll_weight=0.05 if cy is None else 0.0), grip, tilted)
                    if s.ok and abs(s.roll - prev.roll) > math.radians(10):
                        s.ok, s.why = False, "wrist_roll jumps on the way down"
                    if not s.ok:
                        bad = f"{h * 100:.1f} cm: {s.why}"
                        break
                    steps_.append(s)
                    prev = s
                if bad:
                    why = f"descent {bad}"
                    continue
                plan = GraspPlan(centre_xy=centre_xy, cube_yaw=cube_yaw, close_yaw=cy if cy is not None else self.close_yaw(r.q),
                                 hover_h=hh, grasp_h=grasp_h, hover=r, descent=steps_, heights=heights, tilted=tilted)
                break
            if plan is None:
                reasons.append(f"close {math.degrees(cy):+.0f}: {why}" if cy is not None else why)
            else:
                plans.append(plan)
        if not plans:
            return None, reasons

        def score(p):
            s = abs(p.roll) + math.radians(p.grasp.tilt_deg) + 0.01 * p.grasp.pos_err_mm
            if prefer_roll is not None:
                s += 0.5 * abs(p.roll - prefer_roll)
            return s
        return min(plans, key=score), reasons

    def plan_cube(self, centre_xy, cube_yaw, grip, hover_h=0.07, hover_min_h=0.05, grasp_h=0.011,
                  step=0.01, fraction=0.5, prefer_roll=None, close_yaws=None):
        """Grasp plan for a cube (centre xy, yaw) or (None, message)."""
        centre_xy = np.asarray(centre_xy, float)
        heights = list(np.arange(hover_h, hover_min_h - 1e-4, -0.005)) or [hover_h]
        yaws = self.close_options(cube_yaw) if close_yaws is None else close_yaws
        plan, reasons = self.plan_vertical(centre_xy, yaws, grip, heights, grasp_h, step, fraction, cube_yaw, prefer_roll)
        if plan is None and self.allow_tilt:
            plan, more = self.plan_vertical(centre_xy, yaws, grip, heights, grasp_h, step, fraction, cube_yaw,
                                            prefer_roll, tilted=True)
            reasons += [f"tilted: {r}" for r in more]
        if plan is None:
            reach = float(np.linalg.norm(centre_xy))
            msg = (f"no grasp for the cube at ({centre_xy[0] * 100:.1f}, {centre_xy[1] * 100:.1f}) cm, "
                   f"reach {reach * 100:.1f} cm (vertical jaws reach ~30 cm"
                   f"{', tilt up to %.0f deg allowed' % self.max_tilt_deg if self.allow_tilt else ''}): "
                   + " | ".join(reasons[:4]))
            return None, msg
        return plan, ""

    def solve_at(self, plan: GraspPlan, h, grip, fraction=0.5, seed=None) -> IKResult:
        """Same grasp as `plan` (closing direction, tilt mode) at another tip height."""
        wv = WEIGHT_TILTED if plan.tilted else WEIGHT_VERTICAL
        seed = plan.hover.q if seed is None else seed
        return self.check(self.solve(plan.centre_xy, h, plan.close_yaw, grip, fraction, wv, roll_hint=seed[4],
                                     seeds=[seed]), grip, plan.tilted)


# --- camera view poses (autograb.view_pose) ---------------------------------------------------

def load_hand_eye(path) -> np.ndarray:
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)
    r, t = d["rotation"], d["translation"]
    return transform(quaternion_matrix(r["x"], r["y"], r["z"], r["w"]), [t["x"], t["y"], t["z"]])


def view_pose(model: ArmModel, T_gc: np.ndarray, d: float, H: float = 0.20):
    """[lift, elbow, wrist_flex] (pan 0, roll 0) that aim the camera's optical axis at reach d on
    the table from about H above it."""
    lo = np.array([JOINT_LIMITS[n][0] for n in ARM[1:4]])
    hi = np.array([JOINT_LIMITS[n][1] for n in ARM[1:4]])

    def res(x):
        T = model.hand([0.0, x[0], x[1], x[2], 0.0]) @ T_gc
        o, z = T[:3, 3], T[:3, 2]
        if abs(z[2]) < 1e-6:
            return [10.0, 10.0, 10.0]
        hit = o + (model.table_z - o[2]) / z[2] * z
        return [(hit[0] - d) * 100, (o[2] - model.table_z - H) * 100, (z[2] + 1) * 20]
    best = None
    for s in ([-0.4, 0.2, 1.6], [0.0, 0.0, 1.5], [0.3, -0.4, 1.6]):
        r = least_squares(res, np.clip(s, lo + 1e-3, hi - 1e-3), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    return [float(v) for v in best.x]


def default_views(model: ArmModel, T_gc, pans_deg):
    """autograb's search views: near (camera ~20 cm above the table, looking down at ~20 cm),
    close (17 cm out, 18 cm up), far (33 cm out), each at every pan. Joint vectors [rad]."""
    near = list(np.radians([-24.0, 13.0, 94.5]))
    sets = [("near", near)]
    if T_gc is not None:
        sets += [("close", view_pose(model, T_gc, 0.17, 0.18)), ("far", view_pose(model, T_gc, 0.33))]
    return [(f"{tag}{p:+.0f}", np.array([math.radians(p), *v, 0.0])) for tag, v in sets for p in pans_deg]


# --- what the wrist camera sees (overview pose, aimed views) ----------------------------------

@dataclass
class FisheyeCamera:
    """Equidistant (OpenCV fisheye) model of the wrist camera at its calibration size."""
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int

    def project(self, P_cam):
        """(N,2) pixels and (N,) angle off the optical axis [rad] of points in the optical frame."""
        P = np.atleast_2d(np.asarray(P_cam, float))
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        r = np.hypot(x, y)
        th = np.arctan2(r, z)
        k = self.D
        thd = th * (1 + k[0] * th ** 2 + k[1] * th ** 4 + k[2] * th ** 6 + k[3] * th ** 8)
        s = np.where(r > 1e-9, thd / np.maximum(r, 1e-9), 1.0)
        uv = np.c_[self.K[0, 0] * x * s + self.K[0, 2], self.K[1, 1] * y * s + self.K[1, 2]]
        return uv, th


def load_fisheye(path) -> FisheyeCamera:
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)
    return FisheyeCamera(K=np.array(d["camera_matrix"]["data"], float).reshape(3, 3),
                         D=np.array(d["distortion_coefficients"]["data"], float)[:4],
                         width=int(d["image_width"]), height=int(d["image_height"]))


# the work area the overview has to cover: cube centres 15-30 cm out, +-15 cm to the side
WORK_AREA = ((0.15, 0.30), (-0.15, 0.15))
JAW_LINKS = ("gripper_link", "moving_jaw_so101_v1_link")


def camera_pose(model: ArmModel, T_gc: np.ndarray, q_arm) -> np.ndarray:
    """base_link -> wrist_camera_optical_frame (z = optical axis)."""
    return model.hand(q_arm) @ T_gc


def visible(model: ArmModel, T_gc: np.ndarray, cam: FisheyeCamera, q_arm, grip, points, margin_px=10.0,
            marker_m=0.022, min_marker_px=30.0, max_off_axis_deg=85.0, min_view_cos=0.3):
    """Bool mask: the top face of a cube whose top centre is at each of `points` (base_link) is
    seen by the wrist camera at arm pose q_arm, gripper angle `grip` [rad]:

    - inside the image (with `margin_px`) and less than max_off_axis_deg off the optical axis,
    - not behind the jaws (projected convex hulls of gripper_link and the moving jaw: a slight
      over-estimate of what they hide),
    - the top marker (`marker_m`) at least `min_marker_px` wide at the calibration size (1280 px;
      the detector runs at half of it, so 30 -> 15 px) and seen at a view cosine >= min_view_cos.
    """
    from scipy.spatial import Delaunay
    P = np.atleast_2d(np.asarray(points, float))
    T = camera_pose(model, T_gc, q_arm)
    R, o = T[:3, :3], T[:3, 3]

    def proj(X):
        return cam.project((X - o) @ R)[0]

    pc = (P - o) @ R
    uv, th = cam.project(pc)
    ok = ((pc[:, 2] > 0) & (uv[:, 0] > margin_px) & (uv[:, 0] < cam.width - margin_px)
          & (uv[:, 1] > margin_px) & (uv[:, 1] < cam.height - margin_px) & (th < math.radians(max_off_axis_deg)))
    ray = o - P
    ok &= ray[:, 2] / np.maximum(np.linalg.norm(ray, axis=1), 1e-9) >= min_view_cos
    h = marker_m / 2
    side = np.sqrt(np.linalg.norm(proj(P + [h, 0, 0]) - proj(P - [h, 0, 0]), axis=1)
                   * np.linalg.norm(proj(P + [0, h, 0]) - proj(P - [0, h, 0]), axis=1))
    ok &= side >= min_marker_px
    q = model.qdict(q_arm, grip)
    for name, chain, V in model.links:
        if name not in JAW_LINKS:
            continue
        Tl = chain.forward(q)
        Vw = V @ Tl[:3, :3].T + Tl[:3, 3]
        Vw = Vw[(Vw - o) @ R[:, 2] > 0.005]       # in front of the camera
        if len(Vw) >= 4:
            ok &= Delaunay(proj(Vw)).find_simplex(uv) < 0
    return ok


def area_grid(area=WORK_AREA, step=0.01, z=0.0):
    """Points over the area ((x0, x1), (y0, y1)) at height z, and the grid shape (nx, ny)."""
    (x0, x1), (y0, y1) = area
    xs = np.arange(x0, x1 + 1e-6, step)
    ys = np.arange(y0, y1 + 1e-6, step)
    return np.array([[x, y, z] for x in xs for y in ys]), (len(xs), len(ys))


def view_coverage(model: ArmModel, T_gc, cam: FisheyeCamera, q_arm, grip, area=WORK_AREA, step=0.01,
                  cube_edge=0.03, **kw):
    """(fraction of the cube positions in `area` whose top face is seen, mask (nx, ny), grid)."""
    G, shape = area_grid(area, step, model.table_z + cube_edge)
    m = visible(model, T_gc, cam, q_arm, grip, G, **kw)
    return float(m.mean()), m.reshape(shape), G


def overview_pose(model: ArmModel, T_gc, cam: FisheyeCamera, grip=math.radians(12.0), d=0.22, H=0.28,
                  area=WORK_AREA, roll_limit_deg=85.0, wflex_margin_deg=5.0, min_camera_h=0.24,
                  min_clearance=0.06, iters=600, seed=0):
    """Arm pose [rad] from which ONE wrist camera image covers as much of `area` as it can.

    Start: the camera ~H above the table looking straight down at reach d (least squares like
    view_pose, with pan and roll free: the camera sits beside the jaws, so looking straight down
    needs a large wrist_roll). Then a seeded random local search on the coverage itself (the jaws
    hide part of the image, which the start does not know about). Kept: |roll| <= roll_limit_deg,
    wrist_flex wflex_margin_deg inside its limits, camera >= min_camera_h above the table, the
    arm's lowest point >= min_clearance above it. Deterministic for a given seed.
    Returns (q, coverage). Takes ~1 min: run it offline, the node uses `overview_poses_deg`."""
    lo = np.array([JOINT_LIMITS[n][0] for n in ARM])
    hi = np.array([JOINT_LIMITS[n][1] for n in ARM])
    lo[4], hi[4] = -math.radians(roll_limit_deg), math.radians(roll_limit_deg)
    lo[3] += math.radians(wflex_margin_deg)
    hi[3] -= math.radians(wflex_margin_deg)

    def res(x):
        T = camera_pose(model, T_gc, x)
        o, z = T[:3, 3], T[:3, 2]
        hit = o + (model.table_z - o[2]) / min(z[2], -1e-3) * z
        return [(hit[0] - d) * 100, hit[1] * 100, (o[2] - model.table_z - H) * 100, (z[2] + 1) * 20]
    best = None
    for s in ([0.0, -0.6, 0.3, 1.5, -1.4], [0.0, -0.3, 0.0, 1.5, -1.2], [0.0, -0.6, 0.3, 1.5, 1.4]):
        r = least_squares(res, np.clip(s, lo + 1e-3, hi - 1e-3), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r

    def score(q):
        T = camera_pose(model, T_gc, q)
        if T[2, 3] - model.table_z < min_camera_h or model.lowest(q, grip)[1] < min_clearance:
            return -1.0
        return view_coverage(model, T_gc, cam, q, grip, area)[0]
    q = np.clip(best.x, lo, hi)
    s = score(q)
    rng = np.random.default_rng(seed)
    for k in range(iters):
        sigma = math.radians(4.0 if k < iters // 2 else 1.5)
        c = np.clip(q + rng.normal(0.0, sigma, 5), lo, hi)
        sc = score(c)
        if sc > s:
            q, s = c, sc
    return q, s


def search_order(model: ArmModel, T_gc, views, overviews, hint=None, aimed_q=None):
    """The search's view list [(name, q)]: the view aimed at the cube's last known centre `hint`
    (if any), then the overview pose(s), then `views` sorted by the angle between their optical
    axis and `hint` (unsorted without a hint or a hand-eye transform)."""
    views = list(views)
    if hint is not None and T_gc is not None:
        hint = np.asarray(hint, float)

        def off_axis(v):
            T = camera_pose(model, T_gc, v[1])
            d = hint - T[:3, 3]
            return math.acos(float(np.clip(T[:3, 2] @ d / np.linalg.norm(d), -1, 1)))
        views.sort(key=off_axis)
    first = []
    if aimed_q is not None and hint is not None:
        first.append((f"aimed at ({hint[0] * 100:.0f}, {hint[1] * 100:.0f}) cm", np.asarray(aimed_q, float)))
    return first + list(overviews) + views


def aim_view_pose(model: ArmModel, T_gc, target, H=0.22, roll_limit=math.radians(85.0), seed=None,
                  wflex_margin_deg=5.0):
    """Arm pose [rad] with the camera ~H above the table, looking down as far as it can, its
    optical axis through `target` (x, y[, z]) in base_link; wrist_roll pulled towards 0."""
    target = np.asarray(target, float)
    z_t = float(target[2]) if len(target) > 2 else model.table_z
    lo = np.array([JOINT_LIMITS[n][0] for n in ARM])
    hi = np.array([JOINT_LIMITS[n][1] for n in ARM])
    lo[4], hi[4] = -roll_limit, roll_limit
    lo[3] += math.radians(wflex_margin_deg)
    hi[3] -= math.radians(wflex_margin_deg)

    def res(x):
        T = camera_pose(model, T_gc, x)
        o, z = T[:3, 3], T[:3, 2]
        hit = o + (z_t - o[2]) / min(z[2], -1e-3) * z
        return [(hit[0] - target[0]) * 100, (hit[1] - target[1]) * 100, (o[2] - model.table_z - H) * 100,
                (z[2] + 1) * 5, x[4] * 0.5]
    pan = math.atan2(target[1], target[0])
    seeds = [[pan, -0.4, 0.2, 1.5, 0.0], [pan, -0.6, 0.3, 1.5, -1.2], [pan, -0.6, 0.3, 1.5, 1.2]]
    if seed is not None:
        seeds.insert(0, list(seed))
    best = None
    for s in seeds:
        r = least_squares(res, np.clip(s, lo + 1e-3, hi - 1e-3), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    return np.clip(best.x, lo, hi)
