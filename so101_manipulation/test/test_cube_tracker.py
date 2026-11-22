"""Cube estimate window without ROS: robust median, yaw mod 90, stability, `since`."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation.cube_tracker import CubeWindow, TrackedState, cube_key, quarter_median  # noqa: E402


# --- /cubes/tracked -----------------------------------------------------------------------------

def test_tracked_stable_needs_fresh_stamps():
    st = TrackedState()
    for i in range(10):                        # stable filter output, seen before the arm stopped
        st.add(i * 0.1, i * 0.1, [0.22, 0.03, 0.0126], 0.3, 0.6, 1.0, 1.0)
    assert st.estimate("cube_2", now=1.0).stable
    e = st.estimate("cube_2", now=1.5, since=1.2)      # nothing new since the arm stopped
    assert not e.stable
    for i in range(3):                         # three new image stamps after `since`
        st.add(1.3 + i * 0.1, 1.3 + i * 0.1, [0.22, 0.03, 0.0126], 0.3, 0.6, 1.0, 1.0)
    e = st.estimate("cube_2", now=1.6, since=1.2)
    assert e.stable and abs(e.centre[0] - 0.22) < 1e-9


def test_tracked_repeated_stamp_is_not_fresh():
    st = TrackedState()
    for i in range(6):                         # 5 Hz republishing of the same last-seen stamp
        st.add(i * 0.2, 1.0, [0.22, 0.03, 0.0126], 0.0, 0.6, 1.0, 1.0)
    assert st.estimate("cube_2", now=1.2, since=0.5).count == 1
    assert not st.estimate("cube_2", now=1.2, since=0.5).stable


def test_tracked_unstable_filter_but_fresh_observations_agree():
    """The filter still holds old far-view observations (score < 0.5), the fresh close-up ones agree."""
    st = TrackedState()
    for i in range(6):
        st.add(1.0 + i * 0.1, 1.0 + i * 0.1, [0.2003, -0.12, 0.0126], -0.5, 0.3, 2.8, 1.0)
    e = st.estimate("cube_2", now=1.7, since=0.95, min_count=5)
    assert e.stable and abs(e.centre[0] - 0.2003) < 1e-4


def test_tracked_spread_from_covariance_kept():
    st = TrackedState()
    st.add(0.0, 0.0, [0.2, 0.0, 0.0126], 0.0, 0.3, 4.0, 2.0)
    e = st.estimate("cube_1", now=0.1)
    assert not e.stable and e.spread_mm == 4.0


def test_cube_key():
    assert cube_key("cube_3") == cube_key("3") == cube_key(3) == cube_key("K3") == "cube_3"


def test_median_rejects_outlier_and_is_stable():
    w = CubeWindow()
    rng = np.random.default_rng(0)
    for i in range(10):
        w.add(i * 0.1, [0.22 + rng.normal(0, 0.0005), 0.03, 0.0126], math.radians(20))
    w.add(1.0, [0.26, 0.03, 0.0126], math.radians(20))      # one bad frame, 4 cm off
    e = w.estimate("cube_2", now=1.05)
    assert np.linalg.norm(e.centre[:2] - [0.22, 0.03]) < 0.001
    assert e.stable


def test_yaw_wrap_around_45():
    yaw, spread = quarter_median([math.radians(44), math.radians(-44), math.radians(44.5), math.radians(-44.5)])
    assert abs(abs(math.degrees(yaw)) - 45) < 1.0
    assert math.degrees(spread) < 2.0


def test_needs_enough_new_observations():
    w = CubeWindow()
    for i in range(10):
        w.add(i * 0.1, [0.2, 0.0, 0.0126], 0.0)
    e = w.estimate("cube_1", now=1.0, since=0.75)        # only 2 newer than `since`
    assert not e.stable and e.count == 2
    e = w.estimate("cube_1", now=1.0, since=0.35)
    assert e.stable


def test_old_observations_only_are_never_stable():
    w = CubeWindow()
    for i in range(10):
        w.add(i * 0.1, [0.2, 0.0, 0.0126], 0.0)
    e = w.estimate("cube_1", now=5.0, since=4.0)
    assert e is not None and not e.stable and e.age_s > 3.9


def test_spread_too_large_is_not_stable():
    w = CubeWindow()
    for i in range(10):
        w.add(i * 0.1, [0.2 + (0.004 if i % 2 else -0.004), 0.0, 0.0126], 0.0)
    e = w.estimate("cube_1", now=1.0)
    assert not e.stable and e.spread_mm > 3.0
