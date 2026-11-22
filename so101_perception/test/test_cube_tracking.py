"""cube_tracking: the per-cube temporal filter.

    python3 -m pytest so101_perception/test/test_cube_tracking.py -q
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import cube_tracking as ct  # noqa: E402


def obs(t, x, y, yaw_deg, w=0.6, spread=0.0):
    return ct.Observation(t, np.array([x, y, 0.015]), np.radians(yaw_deg), w, spread, 0.0)


def noisy_track(rng, n, centre=(0.30, 0.02), yaw_deg=10.0, sd_mm=0.5, sd_deg=0.5, track=None):
    track = track or ct.CubeTrack()
    for i in range(n):
        track.update(obs(i * 0.1, centre[0] + rng.normal(0, sd_mm / 1000), centre[1] + rng.normal(0, sd_mm / 1000),
                         yaw_deg + rng.normal(0, sd_deg)))
    return track


def test_becomes_stable():
    rng = np.random.default_rng(0)
    track = ct.CubeTrack()
    for i in range(4):
        noisy_track(rng, 1, track=track)
        assert not track.stable and track.confidence < 0.5
    noisy_track(rng, 6, track=track)
    assert track.stable and track.confidence >= 0.5
    assert np.linalg.norm(track.centre[:2] - [0.30, 0.02]) < 0.0006
    assert abs(np.degrees(track.yaw) - 10) < 0.6
    assert track.spread < 0.0015 and track.spread_yaw < np.radians(1.5)


def test_noisy_is_not_stable():
    rng = np.random.default_rng(1)
    track = noisy_track(rng, 20, sd_mm=4.0)
    assert not track.stable and track.confidence < 0.5
    assert track.spread > 0.002


def test_single_outlier_ignored():
    rng = np.random.default_rng(2)
    track = noisy_track(rng, 10)
    before = track.centre.copy()
    assert track.update(obs(5.0, 0.36, 0.02, 10)) == "pending"
    assert not track.stable                      # something odd happened: not trusted for now
    assert track.update(obs(5.1, 0.30, 0.02, 10)) == "added"
    assert track.stable and np.linalg.norm(track.centre - before) < 0.0005


def test_moved_cube_resets():
    rng = np.random.default_rng(3)
    track = noisy_track(rng, 10)
    results = [track.update(obs(10 + i * 0.1, 0.25, -0.05, -30)) for i in range(3)]
    assert results == ["pending", "pending", "reset"]
    assert np.linalg.norm(track.centre[:2] - [0.25, -0.05]) < 1e-6 and track.resets == 1
    assert not track.stable                      # only 3 observations so far
    noisy_track(rng, 3, centre=(0.25, -0.05), yaw_deg=-30, track=track)
    assert track.stable


def test_yaw_wraps_at_45():
    track = ct.CubeTrack()
    for i in range(8):
        track.update(obs(i * 0.1, 0.3, 0.0, 44.0 if i % 2 else -44.0))
    assert abs(abs(np.degrees(track.yaw)) - 45) < 0.1
    assert track.spread_yaw < np.radians(1.5) and track.stable


def test_low_confidence_ignored_and_age():
    track = ct.CubeTrack()
    assert track.update(obs(1.0, 0.3, 0.0, 0, w=0.01)) == "ignored"
    assert track.centre is None and track.age(2.0) == float("inf")
    track.update(obs(1.5, 0.3, 0.0, 0))
    assert abs(track.age(2.0) - 0.5) < 1e-9


def test_tracker_forgets_old_cubes():
    tracker = ct.CubeTracker()
    tracker.update(0, obs(1.0, 0.3, 0.0, 0))
    tracker.update(3, obs(9.0, 0.3, 0.1, 0))
    assert tracker.forget(5.0, now=10.0) == [0]
    assert list(tracker.tracks) == [3]
