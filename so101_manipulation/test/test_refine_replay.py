"""Close-up refinement replayed offline on real wrist-camera frames (no ROS, no arm).

The real detector (so101_perception/scripts/cube_detection.py) runs on the 8 frames of
~/so101_data/cubes/20261122/rest_*.jpg (joint angles saved with them). Its per-frame estimates of a
cube are the "far view" (20 frames, as after an overview). The close-up at the hover is replayed
from the same real per-frame estimates moved by `d`: the view-dependent error of the camera pose
(hand-eye, joint origins, servo sag) that made the far and the close-up view disagree by 6-12 mm on
2026-11-22. Both go through the detector's real filter (cube_tracking.CubeTrack) and the node's
adapter, as on the robot (camera ~12 fps, frames counted from 0.3 s after the arm stopped, the
node from obs_settle_s = 0.4 s).

- BEFORE (the adapter's old fallback, reproduced below): a close-up 4-10 mm from the far estimate
  joins the filter's window of 20 far observations; for ~10 frames the filter's median stays at the
  far position, its outputs agree with each other, and that was taken as a "stable close-up": the
  refinement kept the far position without noticing (the cube ended 4-10 mm off-centre in the jaws).
- NOW (`tracker.fresh`, `PickPlaceNode.close_up`): only per-frame observations taken after the
  settle count; refine_min_obs = 4 that agree within 3 mm give the close-up position, whatever the
  filter still remembers.
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(REPO / "so101_perception" / "scripts"))
sys.path.insert(0, str(REPO / "so101_perception" / "test"))

from so101_manipulation.cube_tracker import CubeWindow, TrackedState  # noqa: E402

DATA = Path(os.path.expanduser("~/so101_data/cubes/20261122"))
FPS = 12.0
GATE_S, SETTLE_S = 0.3, 0.4         # detector motion gate after the arm stops; node obs_settle_s
MIN_OBS, AGREE_MM = 4, 3.0           # refine_min_obs, refine_agree_mm


@pytest.fixture(scope="module")
def frames():
    """Per cube: the real per-frame CubeEstimates of the 8 rest frames."""
    cv2 = pytest.importorskip("cv2")
    if not (DATA / "rest_joints.json").exists():
        pytest.skip(f"no real frames in {DATA}")
    import cube_detection as cd
    import cube_real_frames as rf
    q = json.loads((DATA / "rest_joints.json").read_text())
    R_wc, t_wc, table_z = rf.camera_pose(q)
    geometry = cd.load_cube_geometry(rf.CUBES)
    det = cd.MarkerDetector(refinement="subpix")
    out = {}
    for path in sorted(DATA.glob("rest_*.jpg")):
        gray = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)
        K, D = rf.intrinsics(gray.shape[1])
        for e in cd.cube_estimates(cd.marker_poses(det.detect(gray), K, D, geometry), R_wc, t_wc, table_z=table_z):
            out.setdefault(e.cube, []).append(e)
    assert len(out) >= 5 and all(len(v) == 8 for v in out.values()), {k: len(v) for k, v in out.items()}
    return out


def replay(ests, d_mm, direction=(0.8, 0.6), n_far=20, n_near=40):
    """Far view (n_far frames), a 3 s move, then the hover: the same real per-frame estimates moved
    by d_mm. Yields (t, since, filter CubeTrack, node TrackedState, per-frame CubeWindow) after each
    close-up frame."""
    import cube_tracking as ct
    off = np.r_[np.asarray(direction, float) / np.linalg.norm(direction) * d_mm / 1000, 0.0]
    filt, tracked, raw = ct.CubeTrack(), TrackedState(), CubeWindow()

    def frame(t, e, shift):
        o = ct.Observation(t, e.centre + shift, e.yaw, e.confidence, e.face_spread, e.face_spread_yaw)
        filt.update(o)
        raw.add(t, e.centre + shift, e.yaw, e.confidence)
        tracked.add(t, t, filt.centre, filt.yaw, filt.confidence, filt.spread * 1000, math.degrees(filt.spread_yaw))
    t = 0.0
    for i in range(n_far):
        frame(t, ests[i % len(ests)], 0.0)
        t += 1 / FPS
    arrived = t + 3.0
    since = arrived + SETTLE_S
    t = arrived + GATE_S
    for i in range(n_near):
        frame(t, ests[i % len(ests)], off)
        yield t, since, filt, tracked, raw
        t += 1 / FPS


def old_close_up(t, since, tracked):
    """The adapter before 2026-11-22: the filter's stable flag, or 5 fresh filter OUTPUTS that agree."""
    if t < since:
        return None
    obs_t, centre, yaw, score, *_ = tracked.last
    fresh = [o for o in tracked.window.obs if o[0] >= since]
    if score >= 0.5 and len(fresh) >= 3 and obs_t >= since:
        return centre
    own = tracked.window.estimate("c", t, since, 1.5, 5, 3.0, 4.0)
    return own.centre if own is not None and own.stable else None


def new_close_up(t, since, raw):
    """PickPlaceNode.close_up through DetectionsCubeTracker.fresh."""
    if t < since:
        return None
    e = raw.estimate("c", t, since, math.inf, MIN_OBS, AGREE_MM, 4.0, fresh_only=True)
    return e.centre if e is not None and e.stable else None


@pytest.mark.parametrize("cube", [2, 4])
@pytest.mark.parametrize("d_mm", [0.0, 3.0, 5.0, 7.0, 9.0, 12.0, 15.0])
def test_close_up_follows_the_close_view(frames, cube, d_mm):
    ests = frames[cube]
    far = np.median([e.centre for e in ests], axis=0)
    near = far + np.r_[np.array([0.8, 0.6]) * d_mm / 1000, 0.0]
    for t, since, _, _, raw in replay(ests, d_mm):
        c = new_close_up(t, since, raw)
        if c is not None:
            break
    assert c is not None, "no close-up estimate"
    assert np.linalg.norm(c[:2] - near[:2]) < 0.001, (c, near)
    # refine_min_obs frames after the settle: < 0.5 s at 12 fps (the timeout is 3 s)
    assert t - since < (MIN_OBS + 1) / FPS


@pytest.mark.parametrize("d_mm", [5.0, 7.0, 9.0])
def test_old_adapter_kept_the_far_view(frames, d_mm):
    """Documents the bug: the old fallback 'confirmed' the far position for a close-up 5-9 mm away."""
    ests = frames[2]
    far = np.median([e.centre for e in ests], axis=0)
    got = None
    for t, since, _, tracked, _ in replay(ests, d_mm):
        got = old_close_up(t, since, tracked)
        if got is not None:
            break
    assert got is not None
    assert np.linalg.norm(got[:2] - far[:2]) < 0.0015, "old adapter reported the far position as the close-up"


def test_jump_threshold_close_up_with_dropouts(frames):
    """A close-up right at the filter's jump threshold (10 mm) with every third frame missing (a marker
    lost behind a jaw): the filter alternates between 'held back' and 'added' and is not stable; the
    per-frame close-up still is, at the close view."""
    ests = frames[4]
    far = np.median([e.centre for e in ests], axis=0)
    near = far + np.r_[np.array([0.8, 0.6]) * 0.010, 0.0]
    got, k = None, 0
    for t, since, filt, _, raw in replay(ests, 10.0):
        k += 1
        if k % 3 == 0:
            raw.obs.pop()            # this frame did not show the cube
            continue
        got = new_close_up(t, since, raw)
        if got is not None:
            break
    assert got is not None and np.linalg.norm(got[:2] - near[:2]) < 0.001
