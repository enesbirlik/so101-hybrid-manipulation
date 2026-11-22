"""Tilt sweep for far and near cubes (no ROS): smallest lean, exact grasp point, roll limit,
table clearance, approach and lift along the jaw axis, contact joint, placing a tilted hold."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation.grasp import TILT_MAX_RAISE, GraspPlanner, contact_watch, fold_quarter  # noqa: E402
from so101_manipulation.kinematics import ArmModel, within_limits  # noqa: E402

OPEN = math.radians(30)
EDGE = 0.03
FAR = (0.38, 0.0)


@pytest.fixture(scope="module")
def model():
    return ArmModel()


@pytest.fixture(scope="module")
def planner(model):
    return GraspPlanner(model, max_tilt_deg=45.0, tilt_step_deg=5.0)


@pytest.fixture(scope="module")
def far_plan(planner):
    plan, msg = planner.plan_cube(FAR, 0.0, OPEN)
    assert plan is not None, msg
    return plan


def same(a, b):
    assert len(a) == len(b) and all(np.allclose(x, y) for x, y in zip(a, b))


def gap_point(planner, q):
    Th = planner.m.hand(q)
    return Th[:3, :3] @ planner.hand_point(OPEN) + Th[:3, 3]


def test_far_cube_leans_out(far_plan):
    """38 cm: out of vertical reach, so the jaws lean OUT (tips away from the base) by a multiple of the step."""
    assert far_plan.line is not None and far_plan.tilted
    assert far_plan.lean_deg > 4.0
    assert abs(far_plan.lean_deg / 5.0 - round(far_plan.lean_deg / 5.0)) < 1e-6
    assert abs(far_plan.grasp.lean_deg - far_plan.lean_deg) <= 1.0


def test_smallest_lean_is_chosen(planner, far_plan):
    """Nothing at the next smaller lean (either sign) works for the same cube."""
    smaller = far_plan.lean_deg - 5.0
    heights = list(np.arange(0.07, 0.05 - 1e-4, -0.005))
    plan, _ = planner.plan_tilted(np.array(FAR), planner.close_options(0.0), OPEN, [h - 0.011 for h in heights],
                                  0.011, edge=EDGE, leans_deg=[smaller, -smaller])
    assert plan is None


def test_grasp_point_is_exact_and_pads_at_mid_height(planner, far_plan):
    """Gap point within the IK tolerance of its target: on the jaw axis through the cube centre at
    mid-height (raised at most TILT_MAX_RAISE for the clearance)."""
    g = far_plan.grasp
    assert g.pos_err_mm <= 2.0
    p = gap_point(planner, g.q)
    a = far_plan.line
    c = np.array([FAR[0], FAR[1], planner.m.table_z + EDGE / 2])
    # distance of the cube centre from the jaw axis through the gap point
    v = c - p
    dist = np.linalg.norm(v - (v @ a) * a)
    assert dist <= TILT_MAX_RAISE * math.sin(abs(far_plan.lean)) + 0.0025
    # and the gap point is not above the cube's mid-height
    assert p[2] - planner.m.table_z <= EDGE / 2 + TILT_MAX_RAISE + 0.002


def test_closing_along_a_face_normal_and_roll_limit(planner, far_plan):
    for r in [far_plan.hover] + far_plan.descent + ([far_plan.exit] if far_plan.exit else []):
        assert abs(r.q[4]) <= math.pi / 2 + 1e-6
        assert within_limits(r.q) is None
        assert r.close_err_deg <= planner.close_tol_deg + 1e-6
        assert abs(r.close_elev_deg) <= planner.max_close_elev_deg + 1e-6
    c = planner.close_yaw(far_plan.grasp.q)
    assert abs(math.degrees(fold_quarter(c))) < 3.5           # cube yaw 0: a face normal


def test_table_clearance_along_the_descent(model, far_plan):
    for r in far_plan.descent:
        assert model.lowest(r.q, OPEN)[1] >= 0.003 - 1e-6


def test_descent_follows_the_jaw_axis(planner, far_plan):
    """Hover = back along the approach direction from the grasp point; every descent point on that line."""
    a = far_plan.line
    g = gap_point(planner, far_plan.grasp.q)
    pts = [gap_point(planner, far_plan.hover.q)] + [gap_point(planner, r.q) for r in far_plan.descent]
    for p in pts:
        v = p - g
        assert np.linalg.norm(v - (v @ a) * a) < 0.003         # on the line
        assert v @ a <= 1e-3                                   # behind the grasp point
    travel = -(pts[0] - g) @ a
    assert 0.035 <= travel <= 0.065
    # leaning out: the hover is nearer the base than the grasp, and higher
    assert np.linalg.norm(pts[0][:2]) < np.linalg.norm(g[:2]) - 0.01
    assert pts[0][2] > g[2] + 0.02
    # the descent moves monotonically along the line
    s = [-(p - g) @ a for p in pts]
    assert all(x > y for x, y in zip(s, s[1:]))


def test_lift_backs_out_along_the_line_then_up(planner, far_plan):
    poses = far_plan.retreat_poses()
    n = len(far_plan.descent)
    assert len(poses) == n + (1 if far_plan.exit is not None else 0)
    for q, r in zip(poses[:n - 1], far_plan.descent[::-1][1:]):
        assert np.allclose(q, r.q)
    assert np.allclose(poses[n - 1], far_plan.hover.q)
    if far_plan.exit is not None:
        h = gap_point(planner, far_plan.hover.q)
        e = gap_point(planner, far_plan.exit.q)
        assert np.linalg.norm(e[:2] - h[:2]) < 0.003          # straight up
        assert e[2] > h[2] + 0.015
        assert abs(far_plan.exit.lean_deg - far_plan.lean_deg) <= 1.0   # same orientation (a held cube stays put)
    # without the exit: only back along the line (contact back-up)
    assert len(far_plan.retreat_poses(exit=False)) == n


def test_42_cm_is_reachable(planner):
    """The user's cube at 42 cm (2026-11-22)."""
    plan, msg = planner.plan_cube((0.42, 0.0), 0.0, OPEN)
    assert plan is not None, msg
    assert plan.lean_deg > 20


def test_no_tilt_when_vertical_reaches(planner):
    plan, msg = planner.plan_cube((0.22, 0.03), math.radians(20), OPEN)
    assert plan is not None, msg
    assert plan.line is None and plan.lean == 0.0
    same(plan.retreat_poses(), [r.q for r in plan.descent[::-1][1:]] + [plan.hover.q])


def test_tilt_disabled_keeps_the_old_reach(model):
    p = GraspPlanner(model, allow_tilt=False)
    plan, msg = p.plan_cube(FAR, 0.0, OPEN)
    assert plan is None and "reach 38.0 cm" in msg


def test_lean_angles_order(planner):
    L = planner.lean_angles()
    assert L[:4] == [5.0, -5.0, 10.0, -10.0]
    assert max(abs(x) for x in L) == 45.0


def test_contact_joint(model, planner, far_plan):
    """A push against the descent shows best in shoulder_lift with a positive lag, vertical or leaning."""
    vertical, _ = planner.plan_cube((0.22, 0.0), 0.0, OPEN)
    assert contact_watch(model, vertical) == ("shoulder_lift", 1.0)
    j, s = contact_watch(model, far_plan)
    assert j in ("shoulder_lift", "elbow_flex", "wrist_flex") and s in (1.0, -1.0)
    assert contact_watch(model, far_plan, "elbow_flex")[0] == "elbow_flex"


def test_place_a_tilted_hold_keeps_the_cube_flat(planner, far_plan):
    """Placing with the grasp's lean and roll: the hand orientation equals the grasp's up to a turn about z."""
    target = np.array([0.30, -0.08])
    centre_h = EDGE / 2 + 0.002
    plan, why = planner.plan_place_held(target, centre_h, far_plan.cube_in_hand, far_plan.lean, far_plan.roll, OPEN,
                                        0.059)
    assert plan is not None, why
    R0 = planner.m.hand(far_plan.grasp.q)[:3, :3]
    R1 = planner.m.hand(plan.grasp.q)[:3, :3]
    assert abs(R0[2] @ R1[2] - 1.0) < 1e-3 or np.allclose(R0[2], R1[2], atol=0.02)   # same world-z row
    Th = planner.m.hand(plan.grasp.q)
    c = Th[:3, :3] @ far_plan.cube_in_hand + Th[:3, 3]
    assert np.linalg.norm(c[:2] - target) < 0.002
    assert abs(c[2] - planner.m.table_z - centre_h) < 0.002
    # straight down from the hover
    h = planner.m.hand(plan.hover.q)
    ch = h[:3, :3] @ far_plan.cube_in_hand + h[:3, 3]
    assert np.linalg.norm(ch[:2] - c[:2]) < 0.003 and ch[2] > c[2] + 0.025
    same(plan.retreat_poses(0), [plan.hover.q])
    assert plan.place_tilt_deg == 0.0


def test_place_tilt_fallback_is_bounded(planner):
    """A hold that cannot go down flat lands at most PLACE_MAX_TILT_DEG off flat, on one edge."""
    from so101_manipulation.grasp import PLACE_MAX_TILT_DEG
    hold, msg = planner.plan_cube((0.40, 0.0), 0.0, OPEN)
    assert hold is not None, msg
    grip = math.radians(11.48)
    n = 0
    for t in ([0.30, -0.08], [0.25, 0.0], [0.20, 0.0]):
        plan, why = planner.plan_place_held(np.array(t), EDGE / 2 + 0.002, hold.cube_in_hand, hold.lean, hold.roll,
                                            grip, 0.059)
        if plan is None:
            continue
        n += 1
        assert plan.place_tilt_deg <= PLACE_MAX_TILT_DEG + 1e-6
        assert abs(abs(plan.lean_deg - hold.lean_deg) - plan.place_tilt_deg) < 1e-6
        Th = planner.m.hand(plan.grasp.q)
        c = Th[:3, :3] @ hold.cube_in_hand + Th[:3, 3]
        d = math.radians(plan.place_tilt_deg)
        # the lowest bottom edge of the tilted cube is at the release height
        assert abs(c[2] - planner.m.table_z - EDGE / 2 * (math.cos(d) + math.sin(d)) - 0.002) < 0.0025
    assert n >= 2
    nothing, why = planner.plan_place_held(np.array([0.10, 0.0]), EDGE / 2, hold.cube_in_hand, hold.lean, hold.roll,
                                           grip, 0.059)
    assert nothing is None and "lean" in why
