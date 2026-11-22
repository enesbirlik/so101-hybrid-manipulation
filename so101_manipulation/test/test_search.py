"""Search views and the overview pose, without ROS: coverage of the work area with the fisheye
model and the hand-eye transform, the order of the views, the frame counter, the phase timer."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation.cube_tracker import FrameTimes  # noqa: E402
from so101_manipulation.grasp import (aim_view_pose, camera_pose, default_views, load_fisheye,  # noqa: E402
                                      load_hand_eye, search_order, view_coverage, visible)
from so101_manipulation.kinematics import ArmModel, perception_config_dir, within_limits  # noqa: E402
from so101_manipulation.timing import PhaseTimer  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SEARCH_GRIP = math.radians(12.0)


def node_default(name):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from test_params import node_params
    return node_params()[name]


@pytest.fixture(scope="module")
def model():
    return ArmModel()


@pytest.fixture(scope="module")
def T_gc():
    return load_hand_eye(perception_config_dir() / "wrist_camera_hand_eye.yaml")


@pytest.fixture(scope="module")
def cam():
    return load_fisheye(perception_config_dir() / "wrist_camera_fisheye.yaml")


@pytest.fixture(scope="module")
def overview(model):
    v = node_default("overview_poses_deg")
    assert len(v) % 5 == 0 and v
    return [np.radians(v[k:k + 5]) for k in range(0, len(v), 5)]


def test_overview_covers_the_work_area(model, T_gc, cam, overview):
    """ONE view sees >= 95 % of the cube positions 15-30 cm out, +-15 cm (2026-11-22: 99 %), the top
    marker >= 15 px at the detector's 640 px and not behind the jaws."""
    assert len(overview) == 1
    f, mask, _ = view_coverage(model, T_gc, cam, overview[0], SEARCH_GRIP)
    assert f >= 0.95, f
    # what it misses is next to the base (behind the jaws), never the far half
    nx = mask.shape[0]
    assert mask[nx // 2:].all()


def test_overview_is_better_than_the_old_near_view(model, T_gc, cam, overview):
    near = np.array([0.0, *np.radians([-24.0, 13.0, 94.5]), 0.0])
    f_near = view_coverage(model, T_gc, cam, near, SEARCH_GRIP)[0]
    f_ov = view_coverage(model, T_gc, cam, overview[0], SEARCH_GRIP)[0]
    assert f_ov > f_near + 0.3, (f_ov, f_near)


def test_overview_pose_is_safe(model, T_gc, overview):
    for q in overview:
        assert within_limits(q, math.radians(85.0)) is None
        assert model.lowest(q, SEARCH_GRIP)[1] > 0.045           # transit clearance
        assert model.lowest(q, math.radians(30.0))[1] > 0.045
        h = camera_pose(model, T_gc, q)[2, 3] - model.table_z
        assert 0.22 < h < 0.36, h
        assert math.degrees(q[3]) <= 90.5                         # wrist_flex 5 deg inside its limit


def test_visible_rejects_points_behind_the_camera_and_far_out(model, T_gc, cam, overview):
    q = overview[0]
    pts = np.array([[0.22, 0.0, model.table_z + 0.03], [-0.20, 0.0, model.table_z + 0.03],
                    [0.60, 0.40, model.table_z + 0.03]])
    assert visible(model, T_gc, cam, q, SEARCH_GRIP, pts).tolist() == [True, False, False]


@pytest.mark.parametrize("xy", [(0.16, 0.0), (0.22, 0.10), (0.27, -0.12), (0.20, -0.05)])
def test_aimed_view_sees_the_target(model, T_gc, cam, xy):
    top = np.array([xy[0], xy[1], model.table_z + 0.03])
    ok = False
    for h in (0.22, 0.26, 0.18):           # the node's search_aim_heights
        q = aim_view_pose(model, T_gc, top, h, math.radians(85.0))
        assert within_limits(q, math.radians(85.0)) is None
        if model.lowest(q, SEARCH_GRIP)[1] >= 0.045 and visible(model, T_gc, cam, q, SEARCH_GRIP, top[None])[0]:
            ok = True
            T = camera_pose(model, T_gc, q)
            # the optical axis goes (nearly) through the target
            d = top - T[:3, 3]
            assert math.degrees(math.acos(T[:3, 2] @ d / np.linalg.norm(d))) < 5.0
            break
    assert ok


def test_search_order_aimed_then_overview_then_nearest_views(model, T_gc, overview):
    views = default_views(model, T_gc, [0.0, -25.0, 25.0, -45.0, 45.0])
    ovs = [("overview0", overview[0])]
    hint = np.array([0.20, -0.14, model.table_z + 0.015])       # to the right (-y)
    aimed = aim_view_pose(model, T_gc, hint, 0.22)
    order = search_order(model, T_gc, views, ovs, hint, aimed)
    assert order[0][0].startswith("aimed at (20, -14)")
    assert order[1][0] == "overview0"
    assert len(order) == len(views) + 2
    # the old views follow, the ones whose axis points nearest to the hint first
    rest = order[2:]
    ang = []
    for _, q in rest:
        T = camera_pose(model, T_gc, q)
        d = hint - T[:3, 3]
        ang.append(math.acos(T[:3, 2] @ d / np.linalg.norm(d)))
    assert ang == sorted(ang)


def test_search_order_without_a_hint(model, T_gc, overview):
    views = default_views(model, T_gc, [0.0, 25.0])
    order = search_order(model, T_gc, views, [("overview0", overview[0])], None, None)
    assert [n for n, _ in order] == ["overview0"] + [n for n, _ in views]


def test_frame_times_count_since():
    f = FrameTimes()
    for t in (1.0, 1.1, 1.1, 1.2, 1.3):      # a repeated stamp counts once
        f.add(t)
    assert f.count_since(1.05) == 3
    assert f.count_since(2.0) == 0


def test_phase_timer():
    now = [0.0]
    t = PhaseTimer(clock=lambda: now[0])
    t.enter("search")
    now[0] = 2.0
    t.enter("approach")
    now[0] = 3.0
    t.enter("grasp")
    now[0] = 4.0
    t.enter("approach")          # retry: adds up
    now[0] = 6.5
    t.stop()
    now[0] = 100.0               # stopped: no more time counted
    assert t.totals() == {"search": 2.0, "approach": 3.5, "grasp": 1.0, "total": 6.5}
    assert t.summary() == "search 2.0, approach 3.5, grasp 1.0, total 6.5"
