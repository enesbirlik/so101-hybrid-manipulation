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
- Vertical jaws reach only ~14-30 cm out (~6 cm tip height there, wrist_flex near its limit).
  Elsewhere the tilt sweep (plan_tilted) leans the jaws in the arm's plane, smallest lean first
  (<= max_tilt_deg), with the grasp point exact, the descent and the lift along the jaw axis; or
  the target is rejected.
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
WEIGHT_CLOSE_SOFT = 20.0    # tilt sweep: ~1 mm per 3 deg, so a roll at its +-90 deg bound gives way in the
                            # closing direction (checked against close_tol_deg), not in the grasp position
TILT_MAX_RAISE = 0.006      # tilt sweep: the grasp point rises at most this much to keep the table clearance
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
    lean_deg: float = 0.0          # signed jaw lean in the arm's vertical plane (+: tips away from the base)
    close_elev_deg: float = 0.0    # closing direction above (+) / below (-) the horizontal

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
    heights: list = field(default_factory=list)     # fixed jaw tip above the table, per descent point
    tilted: bool = False
    # explicit tilt sweep (plan_tilted / plan_place_held); lean 0 / line None = the vertical planner
    lean: float = 0.0                 # signed jaw lean [rad] in the arm's vertical plane, + = tips away from the base
    line: np.ndarray | None = None    # unit direction of the descent (base_link); None = straight down
    exit: IKResult | None = None      # lift: after backing out along the line to the hover, straight up to this
    cube_in_hand: np.ndarray | None = None   # cube centre in gripper_link at the grasp (a tilted hold is placed with it)

    @property
    def grasp(self) -> IKResult:
        return self.descent[-1] if self.descent else self.hover

    @property
    def roll(self) -> float:
        return self.hover.roll

    @property
    def lean_deg(self) -> float:
        return math.degrees(self.lean)

    def retreat_poses(self, i=None, exit=True) -> list:
        """Arm poses back out from descent point i (default: the grasp): along the descent line to the
        hover, then (tilted grasps) straight up to `exit`. The lift, the back-off after a miss and the
        place retreat all use this order, so a tilted hand never drags its jaws sideways through the cube."""
        i = len(self.descent) - 1 if i is None else int(i)
        poses = [r.q for r in self.descent[:i][::-1]] + [self.hover.q]
        if exit and self.exit is not None:
            poses.append(self.exit.q)
        return poses

    def summary(self) -> str:
        s = (f"cube ({self.centre_xy[0] * 100:.1f}, {self.centre_xy[1] * 100:.1f}) cm yaw {math.degrees(self.cube_yaw):+.0f}, "
             f"jaws close along {math.degrees(self.close_yaw):+.0f} deg, roll {math.degrees(self.roll):+.0f}, "
             f"hover {self.hover_h * 100:.1f} cm, grasp {self.grasp_h * 100:.1f} cm, "
             f"tilt {self.grasp.tilt_deg:.1f} deg")
        if self.line is not None:
            along = "along the jaw axis" if self.line[2] > -0.999 else "straight down"
            s += (f" (lean {self.lean_deg:+.0f} deg {'outward' if self.lean > 0 else 'inward'}, {along}, "
                  f"closing {self.grasp.close_elev_deg:+.0f} deg off horizontal)")
        elif self.tilted:
            s += " (tilted fallback)"
        return s


class GraspPlanner:
    def __init__(self, model: ArmModel, roll_limit: float = math.pi / 2, ik_tol_mm: float = 2.0,
                 vertical_tol_deg: float = 3.0, max_tilt_deg: float = 45.0, close_tol_deg: float = 3.0,
                 allow_tilt: bool = True, min_clearance: float = 0.003, tilt_step_deg: float = 5.0,
                 max_close_elev_deg: float = 20.0):
        self.m = model
        self.tilt_step_deg = tilt_step_deg
        self.max_close_elev_deg = max_close_elev_deg
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
        # shoulder_pan turns the arm the OTHER way round z: its URDF origin flips the axis (q0 = +0.5
        # points the arm to -29 deg). Azimuth of the arm plane = pan_sign * q0.
        T = model.hand([0.3, 0.0, 0.0, 0.0, 0.0])[:3, 3] - np.r_[model.hand_chain.joints[0][1][:2, 3], 0.0]
        self.pan_sign = 1.0 if math.atan2(T[1], T[0]) * 0.3 > 0 else -1.0

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
            seeds = [[self.pan_sign * pan, *s, roll0] for s in ARM_SEEDS]
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
                  step=0.01, fraction=0.5, prefer_roll=None, close_yaws=None, cube_bottom=0.0, edge=0.03):
        """Grasp plan for a cube (centre xy, yaw) or (None, message): vertical jaws if they reach, else
        the smallest jaw lean of the tilt sweep (allow_tilt). Heights are above the table;
        `cube_bottom` is the cube's bottom above the table (0, or ~edge on another cube)."""
        centre_xy = np.asarray(centre_xy, float)
        heights = list(np.arange(hover_h, hover_min_h - 1e-4, -0.005)) or [hover_h]
        yaws = self.close_options(cube_yaw) if close_yaws is None else close_yaws
        plan, reasons = self.plan_vertical(centre_xy, yaws, grip, heights, grasp_h, step, fraction, cube_yaw, prefer_roll)
        if plan is None and self.allow_tilt:
            # explicit tilt sweep (replaces the weighted "tilted" pass, which preferred missing the
            # cube by 2-4 cm over leaning more than ~3.5 deg)
            travels = [h - grasp_h for h in heights]
            plan, more = self.plan_tilted(centre_xy, yaws, grip, travels, grasp_h, cube_bottom, edge, step,
                                          fraction, cube_yaw, prefer_roll)
            reasons = [f"vertical: {r}" for r in reasons[:2]] + [f"tilted: {r}" for r in more]
        if plan is None:
            reach = float(np.linalg.norm(centre_xy))
            msg = (f"no grasp for the cube at ({centre_xy[0] * 100:.1f}, {centre_xy[1] * 100:.1f}) cm, "
                   f"reach {reach * 100:.1f} cm (vertical jaws reach ~30 cm"
                   f"{', tilt up to %.0f deg allowed' % self.max_tilt_deg if self.allow_tilt else ''}): "
                   + " | ".join(reasons[:5]))
            return None, msg
        return plan, ""

    def solve_at(self, plan: GraspPlan, h, grip, fraction=0.5, seed=None) -> IKResult:
        """Same grasp as `plan` (closing direction, tilt mode) at another tip height."""
        wv = WEIGHT_TILTED if plan.tilted else WEIGHT_VERTICAL
        seed = plan.hover.q if seed is None else seed
        return self.check(self.solve(plan.centre_xy, h, plan.close_yaw, grip, fraction, wv, roll_hint=seed[4],
                                     seeds=[seed]), grip, plan.tilted)


    # --- explicit tilt sweep (far and near cubes) ------------------------------------------
    #
    # A 5-DOF arm keeps the jaw axis in its own vertical plane (pan, then 3 parallel pitch joints,
    # then wrist_roll about the jaw axis). Vertical jaws reach ~14-30 cm; beyond that (and close to
    # the base) the jaws have to LEAN in that plane: tips away from the base for far cubes (the
    # approach comes from the robot's side), towards the base for near ones. The sweep fixes the
    # lean exactly (one residual), the grasp point exactly (three), and the closing direction along
    # a face normal (one, soft within close_tol_deg because |wrist_roll| <= 90 deg can leave it a
    # degree or two short): five equations for five joints. The smallest lean that works wins.

    def hand_point(self, grip, fraction=0.5) -> np.ndarray:
        """Gap point (`fraction` from the fixed to the moving jaw tip at gripper angle `grip`) in gripper_link."""
        tip = self.m.T_hand_tip[:3, 3]
        return tip + fraction * (self.m.jaw_point({"gripper": grip}) - tip)

    @staticmethod
    def lean_of(R, pan) -> float:
        """Signed lean [rad] of the approach (-z of gripper_link) in the vertical plane at `pan`: + = tips away from the base."""
        a = -R[:, 2]
        return math.atan2(a[0] * math.cos(pan) + a[1] * math.sin(pan), -a[2])

    @staticmethod
    def approach_dir(pan, lean) -> np.ndarray:
        """Unit approach direction (where the jaw tips point) for a lean in the vertical plane at `pan`."""
        return np.array([math.sin(lean) * math.cos(pan), math.sin(lean) * math.sin(pan), -math.cos(lean)])

    def pan_origin(self) -> np.ndarray:
        """xy of the shoulder_pan axis in base_link (3.9 cm in front of base_link's origin)."""
        return self.m.hand_chain.joints[0][1][:2, 3]

    def radial_pan(self, xy) -> float:
        o = self.pan_origin()
        return math.atan2(xy[1] - o[1], xy[0] - o[0])

    def solve_pose(self, point_hand, target, lean, close_yaw=None, seeds=None, fix_roll=None,
                   roll_hint=0.0) -> IKResult:
        """Joints that put `point_hand` (gripper_link) at `target` (base_link, exact), lean the jaws by
        `lean` in the arm plane (exact), and close along `close_yaw` (horizontal projection; None =
        free, or `fix_roll`)."""
        m = self.m
        target = np.asarray(target, float)
        point_hand = np.asarray(point_hand, float)
        cdir = None if close_yaw is None else np.array([math.cos(close_yaw), math.sin(close_yaw)])
        lo, hi = self.lo.copy(), self.hi.copy()
        if fix_roll is not None:
            fr = float(np.clip(fix_roll, self.lo[4] + 1e-4, self.hi[4] - 1e-4))   # a roll held at +-90 stays inside
            lo[4], hi[4] = fr - 1e-4, fr + 1e-4

        def res(x):
            Th = m.hand(x)
            R = Th[:3, :3]
            p = R @ point_hand + Th[:3, 3]
            r = list((p - target) * 1000) + [(self.lean_of(R, self.pan_sign * x[0]) - lean) * 1000]
            if cdir is not None:
                c = R @ m.close_dir_local
                ch = c[:2] / max(1e-9, float(np.linalg.norm(c[:2])))
                r += list((ch - cdir) * WEIGHT_CLOSE_SOFT)
            return r

        pan = self.radial_pan(target[:2])
        roll0 = float(np.clip(roll_hint if fix_roll is None else fix_roll, lo[4] + 1e-3, hi[4] - 1e-3))
        if seeds is None:
            seeds = [[self.pan_sign * pan, *s, roll0] for s in ARM_SEEDS]
        best = None
        pad = np.minimum(1e-3, (hi - lo) / 4)
        for s in seeds:
            x0 = np.clip(np.asarray(s, float), lo + pad, hi - pad)
            r = least_squares(res, x0, bounds=(lo, hi), x_scale=1.0, max_nfev=150)
            if best is None or r.cost < best.cost:
                best = r
            if np.linalg.norm(np.asarray(best.fun)[:4]) < 0.5:
                break       # a seed that already fits: no need for the others
        q = best.x
        R = m.hand(q)[:3, :3]
        p = R @ point_hand + m.hand(q)[:3, 3]
        c = R @ m.close_dir_local
        cerr = 0.0
        if cdir is not None:
            ch = c[:2] / max(1e-9, float(np.linalg.norm(c[:2])))
            cerr = math.degrees(math.acos(float(np.clip(ch @ cdir, -1, 1))))
        return IKResult(q=q, pos_err_mm=float(np.linalg.norm(p - target) * 1000), tilt_deg=self.tilt_deg(q),
                        close_err_deg=cerr, lean_deg=math.degrees(self.lean_of(R, self.pan_sign * q[0])),
                        close_elev_deg=math.degrees(math.asin(float(np.clip(c[2], -1, 1)))))

    def at_limits(self, q) -> str:
        names = [n for n, v, a, b in zip(ARM, q, self.lo, self.hi) if min(v - a, b - v) < 1e-3]
        return ", ".join(("wrist_roll at +-%.0f" % math.degrees(self.roll_limit)) if n == "wrist_roll"
                         else f"{n} at its limit" for n in names)

    def check_lean(self, r: IKResult, grip, lean_deg, close=True, min_clearance=None) -> IKResult:
        """Like check(), for a pose of the tilt sweep (lean fixed, position exact)."""
        why = []
        if r.pos_err_mm > self.ik_tol_mm:
            lim = self.at_limits(r.q)
            why.append(f"position off by {r.pos_err_mm:.1f} mm" + (f" ({lim})" if lim else ""))
        if abs(r.lean_deg - lean_deg) > 1.0 or r.tilt_deg > abs(lean_deg) + self.vertical_tol_deg:
            why.append(f"lean {r.lean_deg:+.1f} deg (tilt {r.tilt_deg:.1f}), wanted {lean_deg:+.0f}")
        if close and r.close_err_deg > self.close_tol_deg:
            why.append(f"closing direction off by {r.close_err_deg:.1f} deg ({self.at_limits(r.q) or 'IK'})")
        if close and abs(r.close_elev_deg) > self.max_close_elev_deg:
            why.append(f"closing {r.close_elev_deg:+.0f} deg off horizontal (max {self.max_close_elev_deg:.0f})")
        if abs(r.roll) > self.roll_limit + 1e-6:
            why.append(f"wrist_roll {math.degrees(r.roll):.0f} deg beyond the limit")
        if not why:
            low = self.m.lowest(r.q, grip)[1]
            if low < (self.min_clearance if min_clearance is None else min_clearance):
                why.append(f"lowest point {low * 1000:.1f} mm above the table")
        r.ok, r.why = not why, "; ".join(why)
        return r

    def lean_angles(self):
        """Leans of the sweep [deg], smallest first, both signs: +5, -5, +10, -10, ... up to max_tilt_deg."""
        step = max(0.5, float(self.tilt_step_deg))
        mags = list(np.arange(step, self.max_tilt_deg + 1e-6, step))
        if mags and self.max_tilt_deg - mags[-1] > 1e-6:
            mags.append(self.max_tilt_deg)
        return [s * float(t) for t in mags for s in (1.0, -1.0)]

    def line_poses(self, point_hand, start, direction, lengths, lean, close_yaw, grip, seed, fix_roll=None,
                   min_clearance=None):
        """IK along a straight line start + direction * s for s in `lengths` (in that order), each
        seeded with the previous pose. (poses, None) or (poses so far, why)."""
        out, prev = [], seed
        for s_ in lengths:
            r = self.check_lean(self.solve_pose(point_hand, np.asarray(start) + np.asarray(direction) * s_, lean,
                                                close_yaw, seeds=[prev], fix_roll=fix_roll, roll_hint=prev[4]),
                                grip, math.degrees(lean), close=close_yaw is not None, min_clearance=min_clearance)
            if r.ok and abs(r.roll - prev[4]) > math.radians(10):
                r.ok, r.why = False, "wrist_roll jumps along the line"
            if not r.ok:
                return out, f"{s_ * 100:.1f} cm along the line: {r.why}"
            out.append(r)
            prev = r.q
        return out, None

    def tip_height(self, q) -> float:
        return float(self.m.tip(q)[2] - self.m.table_z)

    def plan_tilted(self, centre_xy, close_yaws, grip, travels, grasp_h, cube_bottom=0.0, edge=0.03, step=0.01,
                    fraction=0.5, cube_yaw=0.0, prefer_roll=None, leans_deg=None):
        """Smallest-lean grasp of the tilt sweep, or (None, reasons).

        Grasp point: the gap point sits on the jaw axis through the cube centre at mid-height, `d`
        beyond it, with `d` chosen so that vertical jaws would put it exactly where the vertical
        planner does (fixed tip at grasp_h); leaning jaws thus keep the pads centred on the cube's
        mid-height. If the lowest point of the hand is below min_clearance there, the grasp point
        rises in 2 mm steps (at most TILT_MAX_RAISE). Hover: `travel` back along the jaw axis (the
        first of `travels` that works); the descent follows that line. Exit (lift): straight up
        from the hover by the height the leaning line gave away, + 2 cm."""
        centre_xy = np.asarray(centre_xy, float)
        pan = self.radial_pan(centre_xy)
        ph = self.hand_point(grip, fraction)
        gz = float(ph[2] - self.m.T_hand_tip[2, 3])      # gap point vs fixed tip along the hand z (vertical jaws)
        pivot_h = cube_bottom + edge / 2
        d = max(0.0, pivot_h - (grasp_h + gz))
        z0 = self.m.table_z
        leans = self.lean_angles() if leans_deg is None else list(leans_deg)
        reasons, last_q = [], {}
        opts = []
        for cy in close_yaws:
            hint = self.roll_for(pan, cy)
            if abs(hint) > self.roll_limit + math.radians(25):
                reasons.append(f"close {math.degrees(cy):+.0f}: needs roll {math.degrees(hint):+.0f} deg")
                continue
            opts.append((cy, float(np.clip(hint, -self.roll_limit, self.roll_limit))))
        if not opts:
            return None, reasons
        best_why, last_err, dropped = {}, {}, set()
        by_mag = {}
        for L in leans:
            by_mag.setdefault(abs(L), []).append(L)
        for mag in sorted(by_mag):
            plans = []
            for L in by_mag[mag]:
                lean = math.radians(L)
                a = self.approach_dir(pan, lean)
                for cy, hint in opts:
                    key = (cy, L > 0)
                    if key in dropped:
                        continue
                    seeds = ([[*last_q[key]]] if key in last_q else []) + [[self.pan_sign * pan, *s, hint] for s in ARM_SEEDS]
                    plan, why = None, ""
                    for raise_ in np.arange(0.0, TILT_MAX_RAISE + 1e-6, 0.002):
                        target = np.array([centre_xy[0], centre_xy[1], z0 + pivot_h + raise_]) + a * d
                        g = self.solve_pose(ph, target, lean, cy, seeds=seeds, roll_hint=hint)
                        if g.pos_err_mm < 5.0:
                            last_q[key] = g.q
                        if raise_ == 0.0:
                            # leaning further this way only moves the grasp point further out of reach
                            prev = last_err.get(key)
                            if prev is not None and prev > 20.0 and g.pos_err_mm > prev + 2.0:
                                dropped.add(key)
                            last_err[key] = g.pos_err_mm
                        g = self.check_lean(g, grip, L)
                        if not g.ok:
                            why = f"grasp: {g.why}"
                            if "lowest point" not in g.why:
                                break           # raising the grasp point does not help
                            seeds = [g.q] + seeds
                            continue
                        cube_c = np.array([centre_xy[0], centre_xy[1], z0 + pivot_h])
                        plan, why = self._line_plan(centre_xy, cube_yaw, cy, grip, ph, target, a, lean, g,
                                                    travels, step, cube_c)
                        break
                    if plan is not None:
                        plans.append(plan)
                    else:
                        best_why[(cy, L > 0)] = f"close {math.degrees(cy):+.0f} lean {L:+.0f}: {why}"
            if plans:
                def score(p):
                    s = abs(p.roll) + 0.5 * math.radians(abs(p.grasp.close_elev_deg)) + 0.01 * p.grasp.pos_err_mm
                    if prefer_roll is not None:
                        s += 0.5 * abs(p.roll - prefer_roll)
                    return s
                return min(plans, key=score), reasons
        return None, reasons + list(best_why.values())

    def _line_plan(self, centre_xy, cube_yaw, cy, grip, ph, target, a, lean, g, travels, step, cube_c):
        """Hover, descent and exit for a tilted grasp pose `g` at `target` (gap point). The descent runs
        along the jaw axis `a` (the jaws slide onto the cube); if that line leaves the workspace (near
        the base wrist_flex runs into its limit), straight down with the jaws leaning as a fallback."""
        why = []
        for line in (a, DOWN):
            for travel in travels:
                if travel <= 0:
                    continue
                ss = list(np.arange(travel, -1e-9, -step))
                if abs(ss[-1]) > 1e-6:
                    ss.append(0.0)
                # solve from the grasp outwards (continuation from a pose that is known to work)
                back, err = self.line_poses(ph, target, -line, ss[::-1][1:], lean, cy, grip, g.q)
                if err:
                    if travel == travels[-1] or travel == min(t for t in travels if t > 0):
                        why.append(f"{'jaw axis' if line is a else 'straight down'}, hover {travel * 100:.1f} cm "
                                   f"back: {err}")
                    continue
                poses = [g] + back                       # grasp ... hover
                hover = poses[-1]
                descent = poses[::-1][1:]                # below the hover ... grasp
                heights = [self.tip_height(r.q) for r in descent]
                exit_ = None
                if line is a:
                    # lift: up from the hover by what the leaning line gave away in height, + 2 cm
                    rise = travel * (1.0 - math.cos(lean)) + 0.02
                    for up in (rise, rise / 2):
                        e, err = self.line_poses(ph, target - a * travel, np.array([0.0, 0.0, 1.0]), [up / 2, up],
                                                 lean, cy, grip, hover.q)
                        if not err:
                            exit_ = e[-1]
                            break
                plan = GraspPlan(centre_xy=centre_xy, cube_yaw=cube_yaw, close_yaw=cy,
                                 hover_h=self.tip_height(hover.q), grasp_h=self.tip_height(g.q), hover=hover,
                                 descent=descent, heights=heights, tilted=True, lean=lean,
                                 line=np.array(line, float), exit=exit_)
                # cube centre in gripper_link (a tilted hold is placed with the same hand orientation)
                Th = self.m.hand(g.q)
                plan.cube_in_hand = Th[:3, :3].T @ (cube_c - Th[:3, 3])
                return plan, ""
        return None, "; ".join(why) or "no hover travel"

    def plan_place_held(self, target_xy, centre_h, cube_in_hand, lean, roll, grip, hover_rise, step=0.01,
                        min_clearance=0.0):
        """Place a cube held by a TILTED grasp with the hand in the same orientation (same lean, same
        wrist_roll: the cube stays flat). The cube centre (`cube_in_hand`) goes over target_xy at
        `centre_h` above the table; hover `hover_rise` straight above; descent straight down.
        (GraspPlan, "") or (None, why)."""
        target_xy = np.asarray(target_xy, float)
        pan = self.radial_pan(target_xy)
        z0 = self.m.table_z
        target = np.array([target_xy[0], target_xy[1], z0 + centre_h])
        seeds = [[self.pan_sign * pan, *s, roll] for s in ARM_SEEDS]
        g = self.check_lean(self.solve_pose(cube_in_hand, target, lean, None, seeds=seeds, fix_roll=roll), grip,
                            math.degrees(lean), close=False, min_clearance=min_clearance)
        if not g.ok:
            return None, f"release pose: {g.why}"
        # straight down onto the target (best for stacking); far out the arm cannot rise at a fixed
        # lean, then along the jaw axis as at the grasp (the cube comes in diagonally, still flat)
        why = []
        for line in (DOWN, self.approach_dir(pan, lean)):
            for rise in np.arange(hover_rise, min(hover_rise, 0.03) - 1e-9, -0.005):    # lower hovers if needed
                travel = rise / max(1e-6, -line[2])
                ss = list(np.arange(step, travel + 1e-9, step))
                if not ss or abs(ss[-1] - travel) > 1e-6:
                    ss.append(travel)
                up, err = self.line_poses(cube_in_hand, target, -line, ss, lean, None, grip, g.q,
                                          fix_roll=roll, min_clearance=min_clearance)
                if not err:
                    poses = [g] + up
                    descent = poses[::-1][1:]
                    return GraspPlan(centre_xy=target_xy, cube_yaw=0.0, close_yaw=self.close_yaw(g.q),
                                     hover_h=self.tip_height(poses[-1].q), grasp_h=self.tip_height(g.q),
                                     hover=poses[-1], descent=descent, heights=[self.tip_height(r.q) for r in descent],
                                     tilted=True, lean=lean, line=np.array(line, float), exit=None,
                                     cube_in_hand=np.asarray(cube_in_hand, float)), ""
            why.append(f"{'straight down' if line is DOWN else 'jaw axis'}: {err}")
        return None, "hover: " + "; ".join(why)



def contact_watch(model: ArmModel, plan: GraspPlan, joint="auto", point_hand=None):
    """(joint, sign) to watch for a contact on the way down `plan.line`.

    A jaw that lands on something feels a force against the descent, F = -f * line. Each servo
    deflects by its torque over its stiffness (the same ST3215 everywhere), so the lag
    (reference - feedback) of joint k is proportional to (J_k . line), J_k = d(tip)/dq_k at the
    grasp pose: the joint with the largest |J_k . line| among lift / elbow / wrist_flex sees the
    contact best, and its sign tells which way the lag goes (+ for shoulder_lift on a vertical
    descent, as the vertical planner has always assumed). `joint` other than "auto": that joint,
    with the sign from the same formula."""
    q = np.asarray(plan.grasp.q, float)
    line = np.asarray(plan.line if plan.line is not None else DOWN, float)
    ph = model.T_hand_tip[:3, 3] if point_hand is None else np.asarray(point_hand, float)

    def pt(x):
        Th = model.hand(x)
        return Th[:3, :3] @ ph + Th[:3, 3]
    p0, eps = pt(q), 1e-5
    proj = {}
    for k in (1, 2, 3):
        dq = np.zeros(5)
        dq[k] = eps
        proj[ARM[k]] = float((pt(q + dq) - p0) @ line / eps)
    name = max(proj, key=lambda n: abs(proj[n])) if joint == "auto" else joint
    v = proj.get(name)
    if v is None:          # pan / roll: no sensible sign from a push along the line
        return name, 1.0
    return name, 1.0 if v >= 0 else -1.0

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
