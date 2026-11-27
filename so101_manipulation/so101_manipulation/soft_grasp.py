"""Soft grasp logic (no ROS): command floor, contact detection, hold target, held check.

The gripper servo is a position loop (P = 16). Its force on the cube grows with the error
between the commanded and the measured angle. A 30 mm paper cube was crushed on the real arm
by a close command far below the cube width, so:

HARD RULE: no gripper command ever goes below the FLOOR, the angle of a jaw gap of
(cube edge - squeeze_mm) (gap ~= 13 mm + 1.35 mm/deg; 30 mm cube, 1.5 mm -> ~11.5 deg). The
closing ramp ENDS at the floor, the hold target is clamped to it, and the motion layer clamps
every gripper goal to it as well. That alone bounds the force (P x the angle between contact and
floor), with or without contact detection and torque limits.

Held or not is decided at the end of the close from the measured angle and the current (the
floor stops the jaws so early that the lag barely moves: see grip_state()).

On top of that:
1. close slowly (closing_speed_deg_s) from the open angle down to the floor;
2. contact detection. Sign convention: the controller's lag (commanded - measured) goes
   NEGATIVE once the jaws are blocked, because the measured angle stays above the falling
   command. Real arm (2026-11-22): free closing -0.3 .. -0.5 deg, at contact (~13.5 deg) -1.2 deg
   growing to -8 deg; load 4 % -> 6-10 %, current 0.01 -> 0.03-0.05 A. This module works with
   blocked = measured - commanded = -(controller lag), which is positive at contact, and reports
   contact once it is `lag_deg` (1.0) above its free-motion baseline for `samples` samples
   (optionally also |load - baseline| > load_pct). A measured angle BELOW the command
   (overshoot) never counts;
3. on contact the ramp is preempted by a hold target max(floor, contact - squeeze_deg);
4. contact above expected + window (the jaws met something wider: a cube edge or corner,
   another cube) aborts. Contact below the window cannot happen: the command stops at the floor.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from statistics import median


@dataclass
class SoftGraspParams:
    closing_speed_deg_s: float = 12.0
    lag_deg: float = 1.0                  # blocked (measured - commanded) above its baseline = contact
    samples: int = 3                      # ... for this many consecutive samples
    load_pct: float = 0.0                 # |load - baseline| that counts as contact; 0 = off
    baseline_window_s: tuple = (1.0, 0.3)  # baseline: median between these many seconds ago
    min_baseline_samples: int = 5
    warmup_factor: float = 2.5            # threshold x this before the running baseline exists
    squeeze_deg: float = 1.5              # hold target = contact - this (never below the floor)
    window_deg: float = 5.0               # contact must be within expected +- this
    empty_deg: float = -9.0               # the gripper's closed stop (informational: commands stop at the floor)
    # held / empty at the end of the close (real arm, 2026-11-22, 30 mm paper cube): command at the
    # floor 11.48 deg -> empty jaws stop at 12.04 (servo deadband 0.56 deg, 6 mA), with the cube at
    # torque limit 20 %: 12.48-12.57 deg, 13-20 mA, held for 10 s
    empty_deadband_deg: float = 0.56      # empty jaws stop this far above the command
    held_angle_margin_deg: float = 0.25   # held: measured > command + deadband + this
    empty_angle_tol_deg: float = 0.15     # empty: measured within this of command + deadband ...
    held_current_ma: float = 12.0         # held: or the current sustained at/above this
    empty_current_ma: float = 10.0        # ... and current below this
    edge_mm: float = 30.0
    squeeze_mm: float = 1.5               # FLOOR = angle for a gap of edge - squeeze_mm
    gap_zero_mm: float = 13.0
    gap_per_deg_mm: float = 1.35


@dataclass
class ContactEvent:
    t: float
    angle_deg: float          # measured angle at detection
    commanded_deg: float
    lag_deg: float            # blocked (measured - commanded) above the baseline
    load: float
    by: str                   # "lag" or "load"


def gap_angle_deg(gap_mm: float, p: SoftGraspParams) -> float:
    return (gap_mm - p.gap_zero_mm) / p.gap_per_deg_mm


def floor_deg(p: SoftGraspParams) -> float:
    """Lowest gripper command allowed while a cube may be between the jaws."""
    return gap_angle_deg(p.edge_mm - p.squeeze_mm, p)


def clamp_command(deg: float, floor: float) -> float:
    return max(deg, floor)


def clamp_gripper_points(points, floor_rad):
    """Gripper trajectory points [[rad], ...] with every command >= floor_rad (None: unchanged).
    Returns (points, clamped?). The motion layer applies this to every gripper goal."""
    if floor_rad is None:
        return [[float(p[0])] for p in points], False
    out = [[max(float(p[0]), floor_rad)] for p in points]
    return out, any(o[0] != float(p[0]) for o, p in zip(out, points))


@dataclass
class ContactDetector:
    p: SoftGraspParams
    static_baseline_lag: float = 0.0      # blocked (measured - commanded) standing still
    static_baseline_load: float = 0.0
    hist: deque = field(default_factory=lambda: deque(maxlen=2000))
    run: int = 0
    peak_load: float = 0.0
    peak_excess: float = 0.0
    # The cube cannot touch the jaws while the command is above expected + window: there the lag only
    # trains the baseline. At torque limit 20 % the jaw trails a 12 deg/s ramp by > 1 deg right after
    # it starts, which counted as contact at 27 deg on 2026-11-22.
    ignore_above_deg: float = float("inf")
    # Two-speed close (close_ramp): only samples commanded at or below this (the start of the slow
    # part) train the running baseline, so the larger tracking lag of the fast part does not
    # inflate it (an inflated baseline would make the contact show late).
    baseline_below_deg: float = float("inf")

    def baseline(self, t: float):
        a, b = self.p.baseline_window_s
        lags = [lag for (ts, lag, _) in self.hist if t - a <= ts <= t - b]
        loads = [ld for (ts, _, ld) in self.hist if t - a <= ts <= t - b and not math.isnan(ld)]
        warm = len(lags) >= self.p.min_baseline_samples
        bl = median(lags) if warm else self.static_baseline_lag
        bload = median(loads) if len(loads) >= self.p.min_baseline_samples else self.static_baseline_load
        # Blocked jaws make `blocked` grow at the ramp speed (12 deg/s), so it is lag_deg above a
        # baseline that trails by >= 0.3 s long before the baseline can follow it.
        return bl, bload, warm

    def update(self, t: float, commanded_deg: float, measured_deg: float, load: float = float("nan")):
        """Feed one sample; returns a ContactEvent once contact is seen, else None."""
        blocked = measured_deg - commanded_deg          # = -(controller lag): positive when blocked
        bl, bload, warm = self.baseline(t)
        excess = blocked - bl
        # until the running baseline exists the servo's tracking lag is unknown: compare with the
        # standing value and a wider threshold
        thr = self.p.lag_deg if warm else self.p.lag_deg * self.p.warmup_factor
        if commanded_deg <= self.baseline_below_deg:
            self.hist.append((t, blocked, load))
        if commanded_deg > self.ignore_above_deg:
            self.run = 0
            return None
        self.peak_excess = max(self.peak_excess, excess)
        if not math.isnan(load):
            self.peak_load = max(self.peak_load, abs(load))
        by = None
        if excess > thr:
            by = "lag"
        elif self.p.load_pct > 0 and not math.isnan(load) and abs(load - bload) > self.p.load_pct:
            by = "load"
        self.run = self.run + 1 if by else 0
        if self.run >= max(1, self.p.samples):
            return ContactEvent(t, measured_deg, commanded_deg, excess, load, by)
        return None


def expected_contact_deg(edge_mm: float, gap_zero_mm: float = 13.0, gap_per_deg_mm: float = 1.35) -> float:
    """Gripper angle at which the jaws touch an object `edge_mm` wide (30 mm cube: ~12.6 deg)."""
    return (edge_mm - gap_zero_mm) / gap_per_deg_mm


def ramp_end_deg(expected: float, p: SoftGraspParams) -> float:
    """Where the closing ramp stops: at the floor (never lower, whatever the window says)."""
    return floor_deg(p)


def close_ramp(start_deg: float, expected: float, p: SoftGraspParams, fast_speed_deg_s: float = 0.0,
               slow_lead_deg: float = 3.5, min_s: float = 0.5):
    """The closing command as [(angle deg, time s)], piecewise linear (sent without velocities on
    the inner point, so the controller interpolates linearly):

      start -> slow_from   at fast_speed_deg_s, only above slow_from = expected + window + slow_lead_deg
                           (contact detection ignores everything above expected + window anyway)
      slow_from -> floor   at closing_speed_deg_s: the whole contact window is crossed slowly

    It ends at the floor and never commands below it. fast_speed_deg_s <= closing_speed_deg_s, or a
    start at or below slow_from, gives the single slow ramp. Returns (points, slow_from)."""
    floor = floor_deg(p)
    slow = max(1e-3, p.closing_speed_deg_s)
    slow_from = max(floor, expected + p.window_deg + max(0.0, slow_lead_deg))
    pts, t, cur = [], 0.0, max(start_deg, floor)
    if fast_speed_deg_s > slow and cur > slow_from + 1e-6:
        t = (cur - slow_from) / fast_speed_deg_s
        pts.append((slow_from, t))
        cur = slow_from
    else:
        slow_from = max(cur, floor)
    t += max(min_s if not pts else 0.0, (cur - floor) / slow)
    pts.append((floor, max(t, min_s)))
    return pts, slow_from


def ramp_trajectory(start_deg: float, pts, p: SoftGraspParams, step_deg: float = 1.0, last_stretch: float = 1.5):
    """close_ramp points as trajectory points [(angle deg, velocity deg/s, time s)] with EXPLICIT
    velocities, so the controller's cubic segments cannot overshoot (positions only, the inner
    points were interpolated with an overshoot below the floor on mock hardware):

    - the slow part is cut into `step_deg` pieces whose inner points carry the slow speed: the
      cubic between two points with the segment's own mean speed at both ends is a straight line;
    - the last piece ends at rest at the floor and takes last_stretch times longer: a cubic from
      v to 0 whose mean is v/last_stretch is monotonic (no overshoot) and peaks at ~0.9 v;
    - the fast part (if any) ends at slow_from with the slow speed, monotonic as well.
    Every angle is >= the floor and the angles decrease monotonically."""
    floor = floor_deg(p)
    slow = max(1e-3, p.closing_speed_deg_s)
    out = []
    t0 = 0.0
    cur = max(start_deg, floor)
    if len(pts) == 2:
        a, t = pts[0]
        out.append((a, -slow, t))
        cur, t0 = a, t
    if cur - floor < 1e-6:
        return out + [(floor, 0.0, max(t0 + 0.1, pts[-1][1]))]
    n = max(1, int(math.ceil((cur - floor) / step_deg - 1e-9)))
    seg = (cur - floor) / n
    t = t0
    for k in range(1, n + 1):
        a = cur - k * seg
        last = k == n
        from_rest = k == 1 and len(pts) == 1          # no fast part: the first piece starts at rest
        t += seg / slow * (last_stretch if (last or from_rest) else 1.0)
        out.append((floor if last else a, 0.0 if last else -slow, t))
    if n == 1 and len(pts) == 1 and out[-1][2] < pts[-1][1]:
        out[-1] = (floor, 0.0, pts[-1][1])
    return out


# --- per-object grasp parameters (wooden cubes of different sizes) ------------------------------
#
# The gap model (gap ~= 13 mm + 1.35 mm/deg) was measured up to 30 deg. Beyond that it is an
# extrapolation: the jaw-mesh tips of the URDF give a gap that stays ~5 mm WIDER than the model up to
# 50 deg (40 deg: mesh 72 mm, model 67; 50 deg: 84 vs 80.5) and about equal at 60 deg. So an opening
# computed with the model is at least as wide as asked for up to ~55 deg: good enough for the
# approach opening, which only needs to be wide enough. Contact angles and the floor of cubes up to
# ~50 mm lie at <= 28 deg, inside the measured range.

MAX_SQUEEZE_MM = 6.0     # the floor never sits more than this below the object width, whatever the profile says


@dataclass
class GripProfile:
    """How hard to grip (paper: soft, wood: rigid and heavier)."""
    name: str = "paper"
    torque_limit: float = 200.0      # 0.1 % units
    squeeze_deg: float = 1.5
    squeeze_mm: float = 1.5
    closing_speed_deg_s: float = 6.0
    size_uncertainty_mm: float = 0.0  # vision size error: widens the contact window


def gap_mm(angle_deg: float, p: SoftGraspParams) -> float:
    """Jaw gap at a gripper angle (inverse of gap_angle_deg): the MEASURED width at a contact."""
    return p.gap_zero_mm + p.gap_per_deg_mm * angle_deg


def approach_open_deg(width_mm: float, open_deg: float, side_mm: float, max_open_deg: float,
                      gap_zero_mm: float = 13.0, gap_per_deg_mm: float = 1.35) -> float:
    """Approach opening for an object `width_mm` wide: max(open_deg, angle of width + 2 x side_mm),
    at most max_open_deg (extrapolated gap model above 30 deg, see above)."""
    need = (width_mm + 2 * side_mm - gap_zero_mm) / gap_per_deg_mm
    return float(min(max_open_deg, max(open_deg, need)))


def params_for_width(base: SoftGraspParams, width_mm: float, profile: GripProfile | None = None) -> SoftGraspParams:
    """SoftGraspParams for one grasp of an object `width_mm` wide between the jaws:
    floor = angle(width - squeeze_mm) (HARD RULE, never lower), expected contact = angle(width),
    contact window widened by the size uncertainty; the profile sets squeeze and closing speed."""
    from dataclasses import replace
    pr = profile or GripProfile(squeeze_deg=base.squeeze_deg, squeeze_mm=base.squeeze_mm,
                                closing_speed_deg_s=base.closing_speed_deg_s)
    return replace(base, edge_mm=float(width_mm), squeeze_deg=pr.squeeze_deg,
                   squeeze_mm=min(max(0.0, pr.squeeze_mm), MAX_SQUEEZE_MM),
                   closing_speed_deg_s=pr.closing_speed_deg_s,
                   window_deg=base.window_deg + max(0.0, pr.size_uncertainty_mm) / base.gap_per_deg_mm)


def in_window(contact_deg: float, expected: float, p: SoftGraspParams) -> bool:
    return abs(contact_deg - expected) <= p.window_deg


def hold_target_deg(contact_deg: float, p: SoftGraspParams) -> float:
    return clamp_command(contact_deg - p.squeeze_deg, floor_deg(p))


HELD, EMPTY, UNCERTAIN = "held", "empty", "uncertain"


def grip_state(measured_deg: float, command_deg: float, p: SoftGraspParams, current_ma: float = float("nan")):
    """(HELD | EMPTY | UNCERTAIN, why) of a gripper standing at `command_deg` (the floor or the hold
    target). An empty gripper stops `empty_deadband_deg` above its command (servo deadband):

      held      measured > command + deadband + held_angle_margin, or current >= held_current_ma
      empty     measured <= command + deadband + empty_angle_tol and current < empty_current_ma
      uncertain anything else (treated as not held)

    `current_ma` is the mean over a short window (NaN when the hardware has no current: mock)."""
    empty_at = command_deg + p.empty_deadband_deg
    cur = "" if math.isnan(current_ma) else f", {current_ma:.0f} mA"
    if measured_deg > empty_at + p.held_angle_margin_deg:
        return HELD, f"jaws stopped at {measured_deg:.2f} deg, {measured_deg - empty_at:+.2f} over empty ({empty_at:.2f}){cur}"
    if not math.isnan(current_ma) and current_ma >= p.held_current_ma:
        return HELD, f"current {current_ma:.0f} mA >= {p.held_current_ma:.0f} (jaws at {measured_deg:.2f} deg)"
    # at (or even below) where empty jaws stop, with little current: nothing between the jaws
    if measured_deg <= empty_at + p.empty_angle_tol_deg and (math.isnan(current_ma) or current_ma < p.empty_current_ma):
        return EMPTY, f"jaws at {measured_deg:.2f} deg = empty ({empty_at:.2f}){cur}: nothing between them"
    return UNCERTAIN, f"jaws at {measured_deg:.2f} deg (empty {empty_at:.2f}){cur}: neither held nor clearly empty"


def held(measured_deg: float, command_deg: float, p: SoftGraspParams, current_ma: float = float("nan")):
    """(held?, why): grip_state == HELD."""
    state, why = grip_state(measured_deg, command_deg, p, current_ma)
    return state == HELD, why


def simulated_measurement(commanded_deg: float, actual_deg: float, sim_contact_deg: float) -> float:
    """Mock hardware has no contact: pretend a cube blocks the jaws at sim_contact_deg."""
    if math.isnan(sim_contact_deg):
        return actual_deg
    return max(actual_deg, sim_contact_deg)
