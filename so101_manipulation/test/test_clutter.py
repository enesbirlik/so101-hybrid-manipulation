"""Other cubes as obstacles (clutter.py) with the real planner and the jaw meshes, no ROS.

The 2026-11-22 case: K2 picked with K4 standing 4.7 cm away; the open jaws (53 mm gap + jaw
thickness) land on a neighbour that stands along the closing direction."""

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation import clutter as cl  # noqa: E402
from so101_manipulation.grasp import GraspPlanner  # noqa: E402
from so101_manipulation.kinematics import ArmModel  # noqa: E402

MARGIN = 0.005
MIN_OPEN = cl.min_open_deg(30.0, 8.0, 13.0, 1.35)     # 24.4 deg: gap 46 mm
CUBE = np.array([0.22, 0.0])


@pytest.fixture(scope="module")
def model():
    return ArmModel()


@pytest.fixture(scope="module")
def planner(model):
    return GraspPlanner(model)


@pytest.fixture(scope="module")
def foot(model):
    return cl.HandFootprint(model)


def at(direction_deg, dist=0.047, name="cube_4", top=0.03, centre=CUBE):
    a = math.radians(direction_deg)
    return cl.Obstacle(name, np.asarray(centre) + dist * np.array([math.cos(a), math.sin(a)]), 0.0, top)


def among(planner, foot, yaw_deg, obstacles, centre=CUBE, **kw):
    def fn(o, acc):
        return planner.plan_cube(np.asarray(centre), math.radians(yaw_deg), math.radians(o), accept=acc, **kw)
    t0 = time.monotonic()
    plan, msg, accepts = cl.plan_among(fn, foot, obstacles, 30.0, MIN_OPEN, MARGIN)
    return plan, msg, accepts, time.monotonic() - t0


# --- geometry -----------------------------------------------------------------------------

def test_poly_distance():
    a = cl.square([0, 0], 0.0, 0.03)
    assert cl.poly_distance(a, cl.square([0.05, 0], 0.0, 0.03)) == pytest.approx(0.02)
    assert cl.poly_distance(a, cl.square([0.04, 0.04], 0.0, 0.03)) == pytest.approx(math.hypot(0.01, 0.01))
    assert cl.poly_distance(a, cl.square([0.02, 0], 0.0, 0.03)) == pytest.approx(-0.01)
    assert cl.poly_distance(a, cl.square([0.0, 0.0], math.radians(45), 0.03)) < 0
    h = cl.hull2d(np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5]], float))
    assert len(h) == 4


def test_open_jaws_meet_a_neighbour_along_the_closing_direction(planner, foot):
    """Cube at 22 cm, yaw 0: the only closing direction is along x (+-90 need |roll| > 90). A neighbour
    4.7 cm away along it is under a jaw (moving jaw +x, fixed jaw / body -x); across, 2+ cm clear."""
    grip = math.radians(30.0)
    plan, _ = planner.plan_vertical(CUBE, [math.pi], grip, [0.07], 0.011)
    poses = plan.poses()
    assert foot.clearance(poses, grip, [at(0)])[0] < MARGIN
    assert foot.clearance(poses, grip, [at(180)])[0] < 0
    assert foot.clearance(poses, grip, [at(90)])[0] > 0.015
    assert foot.clearance(poses, grip, [at(-90)])[0] > 0.015
    # a lower cube the jaws pass over is no obstacle; the same cube 3 levels high is
    assert foot.clearance(poses, grip, [at(90, 0.035, top=0.03)])[0] > MARGIN
    c3 = foot.clearance(poses, grip, [at(90, 0.035, top=0.09)])
    assert c3[0] < MARGIN and c3[2] in ("wrist_camera_mount_link", "wrist_camera_link", "gripper_link",
                                        "moving_jaw_so101_v1_link")


# --- picks ------------------------------------------------------------------------------------

def test_no_neighbour_is_the_old_plan(planner, foot):
    plan, _, accepts, _ = among(planner, foot, 20.0, [])
    ref, _ = planner.plan_cube(CUBE, math.radians(20.0), math.radians(30.0))
    assert plan.open_deg == 30.0 and not accepts
    assert np.allclose(plan.grasp.q, ref.grasp.q)


@pytest.mark.parametrize("side", [0, 90, 180, 270])
def test_closing_direction_turns_away_from_the_neighbour(planner, foot, side):
    """Yaw 20: two closing directions are reachable (-160 at roll +23, +110 at roll -76). With K4 4.7 cm
    away on any side, the one whose jaws stay clear wins, at full opening."""
    ob = at(20 + side)
    plan, msg, accepts, dt = among(planner, foot, 20.0, [ob])
    assert plan is not None, msg
    assert plan.open_deg == 30.0 and plan.clutter_m >= MARGIN and plan.clutter_with == "cube_4"
    # the jaws close across the line to the neighbour, not along it
    along = abs(math.cos(plan.close_yaw - math.radians(20 + side)))
    assert along < 0.1, math.degrees(plan.close_yaw)
    assert accepts[0].stats["rejected"] >= 1
    assert dt < 2.0


def test_smaller_opening_when_no_direction_clears(planner, foot):
    """Yaw 0 (one closing direction) and a neighbour 5.2 cm away on the moving jaw's side: the full
    opening leaves < 5 mm, a smaller one (still >= cube + 2 x 8 mm) clears."""
    plan, msg, accepts, dt = among(planner, foot, 0.0, [at(0, 0.052)])
    assert plan is not None, msg
    assert MIN_OPEN - 0.1 <= plan.open_deg < 30.0 and plan.clutter_m >= MARGIN
    assert dt < 4.0


def test_refusal_names_the_neighbour(planner, foot):
    obs = [at(0, name="cube_4"), at(90, name="cube_5"), at(-90, name="cube_3")]
    plan, msg, accepts, dt = among(planner, foot, 0.0, obs)
    assert plan is None
    why = cl.refusal("no clear grasp of cube_2", CUBE, accepts, MARGIN, MIN_OPEN)
    assert "cube_4" in why and "4.7 cm away" in why and "every closing direction" in why, why
    assert dt < 6.0, dt


def test_real_k2_k4_positions(planner, foot):
    """K2 and K4 where the detector put them on the real rest-pose frames (2026-11-22, 4.9 cm apart):
    the plan the planner chose alone closes along the line to K4; with K4 known it takes another."""
    k2, k4 = np.array([0.3017, 0.0147]), np.array([0.3429, 0.0410])
    alone, _ = planner.plan_cube(k2, math.radians(-3.9), math.radians(30.0))
    ob = cl.Obstacle("cube_4", k4, math.radians(0.8), 0.03)
    assert foot.clearance(alone.poses(), math.radians(30.0), [ob])[0] < MARGIN
    plan, msg, _, dt = among(planner, foot, -3.9, [ob], centre=k2)
    assert plan is not None, msg
    assert plan.clutter_m >= MARGIN and dt < 4.0


# --- place --------------------------------------------------------------------------------------

def test_place_checks_the_cube_and_the_release_opening(planner, foot):
    hold = math.radians(11.5)
    plan, _ = planner.plan_vertical(CUBE, [None], hold, [0.071], 0.013, prefer_roll=0.0)
    # the placed cube itself would touch a neighbour 3.2 cm away
    acc = cl.make_accept(foot, [at(90, 0.032)], [11.5], MARGIN, release_deg=[30.0, 27.2, MIN_OPEN])
    ok, why = acc(plan)
    assert not ok and "cube would land" in why
    # a neighbour across the closing direction, far enough: the full release opening
    c = math.degrees(plan.close_yaw)
    acc = cl.make_accept(foot, [at(c + 90, 0.05)], [11.5], MARGIN, release_deg=[30.0, 27.2, MIN_OPEN])
    ok, why = acc(plan)
    assert ok and plan.release_deg == 30.0, why
    # along the closing direction: the release opens as wide as stays clear, never into the neighbour
    seen = set()
    for side in (0.0, 180.0):
        for dist in (0.046, 0.048, 0.050, 0.052, 0.056):
            ob = [at(c + side, dist)]
            clear = {r: foot.clearance(plan.poses(), math.radians(r), ob, MARGIN)[0] >= MARGIN
                     for r in (30.0, 27.2, MIN_OPEN)}
            hold_clear = foot.clearance(plan.poses(), hold, ob, MARGIN)[0] >= MARGIN
            ok, why = cl.make_accept(foot, ob, [11.5], MARGIN, release_deg=[30.0, 27.2, MIN_OPEN])(plan)
            want = next((r for r in (30.0, 27.2, MIN_OPEN) if clear[r]), None)
            if not hold_clear or want is None:
                assert not ok, (side, dist, why)
                seen.add("refused")
            else:
                assert ok and plan.release_deg == want, (side, dist, why)
                seen.add("full" if want == 30.0 else "smaller")
    assert "smaller" in seen or "refused" in seen, seen
