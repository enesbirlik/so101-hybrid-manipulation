"""Grasp geometry without ROS: gap-midpoint correction, yaw options, roll limit, reach."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation.grasp import GraspPlanner, fold_quarter, wrap  # noqa: E402
from so101_manipulation.kinematics import ArmModel, within_limits  # noqa: E402

OPEN = math.radians(30)


@pytest.fixture(scope="module")
def model():
    return ArmModel()


@pytest.fixture(scope="module")
def planner(model):
    return GraspPlanner(model)


def gap_point(model, q, grip=OPEN, frac=0.5):
    return model.tip(q) + model.gap_offset(q, grip, frac)


def test_gap_midpoint_is_beside_the_fixed_tip(model, planner):
    """At 30 deg open the gap midpoint is ~3 cm from gripper_frame_link (the fixed jaw tip)."""
    plan, msg = planner.plan_cube([0.22, 0.03], math.radians(20), OPEN)
    assert plan is not None, msg
    off = model.gap_offset(plan.grasp.q, OPEN)
    assert 0.02 < np.linalg.norm(off[:2]) < 0.04
    # and that is why the correction matters: the fixed tip itself is far off the cube centre
    assert np.linalg.norm(model.tip(plan.grasp.q)[:2] - [0.22, 0.03]) > 0.02


@pytest.mark.parametrize("xy,yaw", [((0.22, 0.03), 20), ((0.20, -0.12), -30), ((0.15, 0.15), 44), ((0.28, 0.05), 0)])
def test_gap_midpoint_over_cube_centre(model, planner, xy, yaw):
    plan, msg = planner.plan_cube(xy, math.radians(yaw), OPEN)
    assert plan is not None, msg
    for h, r in zip(plan.heights, plan.descent):
        g = gap_point(model, r.q)
        assert np.linalg.norm(g[:2] - xy) < 0.002          # gap midpoint over the centre
        assert abs(model.tip(r.q)[2] - model.table_z - h) < 0.002
    assert abs(model.tip(plan.grasp.q)[2] - model.table_z - 0.011) < 0.002
    assert plan.grasp.tilt_deg < 3.0                         # vertical jaws
    assert model.lowest(plan.grasp.q, OPEN)[1] > 0.003       # nothing touches the table


def test_gap_fraction_moves_the_cube_towards_the_fixed_jaw(model, planner):
    p_mid, _ = planner.plan_cube([0.22, 0.0], 0.0, OPEN, fraction=0.5)
    p_near, _ = planner.plan_cube([0.22, 0.0], 0.0, OPEN, fraction=0.3)
    d_mid = np.linalg.norm(model.tip(p_mid.grasp.q)[:2] - [0.22, 0.0])
    d_near = np.linalg.norm(model.tip(p_near.grasp.q)[:2] - [0.22, 0.0])
    assert d_near < d_mid


@pytest.mark.parametrize("yaw", [-44, -30, -10, 0, 10, 20, 35, 44])
def test_jaws_close_along_a_face_normal(planner, yaw):
    """The closing direction is one of the 4 face normals, with the smallest |roll| available."""
    xy = (0.22, 0.03)
    plan, msg = planner.plan_cube(xy, math.radians(yaw), OPEN)
    assert plan is not None, msg
    c = planner.close_yaw(plan.grasp.q)
    assert abs(math.degrees(fold_quarter(c - math.radians(yaw)))) < 3.0
    # no other option has a clearly smaller roll
    pan = math.atan2(xy[1], xy[0])
    rolls = [abs(planner.roll_for(pan, cy)) for cy in planner.close_options(math.radians(yaw))]
    assert abs(plan.roll) <= min(rolls) + math.radians(8)
    assert abs(plan.roll) <= math.radians(45) + math.radians(8)   # 4 options: one is always within 45 deg


def test_four_options_are_90_deg_apart(planner):
    opts = planner.close_options(math.radians(20))
    for a, b in zip(opts, opts[1:]):
        assert abs(abs(math.degrees(wrap(b - a))) - 90) < 1e-6


def test_roll_limit_never_exceeded(model, planner):
    rng = np.random.default_rng(3)
    for _ in range(10):
        r, th = rng.uniform(0.15, 0.29), rng.uniform(-1.0, 1.0)
        xy = (r * math.cos(th), r * math.sin(th))
        plan, msg = planner.plan_cube(xy, rng.uniform(-math.pi / 4, math.pi / 4), OPEN)
        if plan is None:
            continue
        for x in [plan.hover] + plan.descent:
            assert abs(x.q[4]) <= math.pi / 2 + 1e-6
            assert within_limits(x.q) is None


def test_restricted_roll_limit_drops_options(model):
    tight = GraspPlanner(model, roll_limit=math.radians(20))
    plan, msg = tight.plan_cube([0.22, 0.0], math.radians(40), OPEN)
    if plan is not None:
        assert abs(plan.roll) <= math.radians(20) + 1e-6


def test_within_limits_refuses_roll_beyond_90():
    assert within_limits([0, 0, 0, 0, math.radians(95)]) is not None
    assert within_limits([0, 0, 0, 0, math.radians(-124)]) is not None
    assert within_limits([0, 0, 0, 0, math.radians(89)]) is None


def test_out_of_reach_is_rejected_with_a_clear_message(planner):
    plan, msg = planner.plan_cube([0.40, 0.0], 0.0, OPEN)
    assert plan is None
    assert "reach 40.0 cm" in msg


def test_vertical_reach_limit_and_tilt_fallback(model):
    vertical_only = GraspPlanner(model, allow_tilt=False)
    with_tilt = GraspPlanner(model, allow_tilt=True, max_tilt_deg=15)
    p_near, _ = vertical_only.plan_cube([0.25, 0.0], 0.0, OPEN)
    assert p_near is not None and not p_near.tilted
    p_far_v, msg = vertical_only.plan_cube([0.34, 0.0], 0.0, OPEN)
    p_far_t, _ = with_tilt.plan_cube([0.34, 0.0], 0.0, OPEN)
    if p_far_v is None:
        assert "reach 34.0 cm" in msg
    if p_far_t is not None:
        assert p_far_t.grasp.tilt_deg <= 15.0 + 1e-6


def test_path_clearance_detects_a_dip(model):
    hover = GraspPlanner(model).plan_cube([0.22, 0.0], 0.0, OPEN)[0]
    a, b = hover.hover.q, hover.grasp.q
    assert model.path_clearance(a, a, OPEN) > 0.05
    assert model.path_clearance(a, b, OPEN) < 0.012
    # a carried cube hanging 11 mm under the tip lowers the clearance by about that
    assert model.path_clearance(a, a, OPEN, load_below=0.011) < model.path_clearance(a, a, OPEN)


def test_clamp_arm_keeps_a_biased_goal_inside_the_limits():
    """2026-11-22: hover trim +0.2 deg on a wrist_flex planned at its 95 deg limit was refused."""
    import math
    from so101_manipulation.motion import clamp_arm
    from so101_manipulation.kinematics import JOINT_LIMITS, within_limits
    q = [0.1, -0.2, 0.3, JOINT_LIMITS["wrist_flex"][1] + math.radians(0.2), math.radians(95.0)]
    c = clamp_arm(q, roll_limit=math.pi / 2)
    assert within_limits(c) is None
    assert abs(c[3] - JOINT_LIMITS["wrist_flex"][1]) < 1e-12 and abs(c[4] - math.pi / 2) < 1e-12
    assert list(c[:3]) == q[:3]
