"""Per-cube temporal filter of the cube detector's estimates, in base_link (no ROS).

The camera moves with the arm, so the estimates are fused in the robot's base frame: a cube that
lies still has the same centre and yaw whatever the arm does, and views from several arm poses
average out each other's errors (corner noise, a face's printing error, joint and hand-eye error).

Per cube, CubeTrack keeps the last `window` observations that agree with each other:

- estimate: weighted median of x and y (robust to a stray frame), height likewise; yaw is the
  weighted circular mean modulo 90 deg (on the circle of 4x the angle, so -44 and +44 deg average
  to +-45, not 0).
- gating: an observation farther than `jump_mm` / `jump_deg` from the estimate is held back as a
  candidate. `confirm` candidates in a row that agree with each other mean the cube was moved:
  the track restarts from them. A single stray frame is dropped once a good one arrives.
- stable: at least `min_obs` observations, spread within `stable_mm` and `stable_deg`, and no
  candidates pending (nothing suggests the cube just moved).
- spread: RMS distance (xy) of the observations from the estimate, and RMS yaw deviation; the
  per-frame face disagreement is folded in, so one consistent face does not look perfect.
"""

from collections import deque
from dataclasses import dataclass

import numpy as np

from cube_detection import circular_mean_90, fold_yaw


@dataclass
class Observation:
    stamp: float              # s
    centre: np.ndarray        # m, base_link
    yaw: float                # rad, folded
    weight: float = 1.0       # per-frame confidence
    face_spread: float = 0.0  # m
    face_spread_yaw: float = 0.0


@dataclass
class TrackerParams:
    window: int = 20           # observations kept per cube
    jump_mm: float = 10.0      # farther than this from the estimate: the cube may have moved
    jump_deg: float = 12.0
    confirm: int = 3           # this many agreeing jumped observations in a row restart the track
    min_obs: int = 5           # observations needed before a track can be stable
    stable_mm: float = 2.0     # spread limits for stable
    stable_deg: float = 3.0
    min_weight: float = 0.05   # per-frame estimates with a lower confidence are ignored


def _weighted_median(x, w):
    i = np.argsort(x)
    c = np.cumsum(w[i])
    return float(x[i][np.searchsorted(c, c[-1] / 2)])


class CubeTrack:
    def __init__(self, params=None):
        self.p = params or TrackerParams()
        self.obs = deque(maxlen=self.p.window)
        self.pending = []
        self.resets = 0
        self.centre, self.yaw = None, 0.0
        self.spread, self.spread_yaw = float("inf"), float("inf")
        self.last_stamp = None

    # -- update -------------------------------------------------------------------------
    def _close(self, a_centre, a_yaw, b_centre, b_yaw):
        return (np.linalg.norm(a_centre[:2] - b_centre[:2]) <= self.p.jump_mm / 1000
                and abs(fold_yaw(a_yaw - b_yaw)) <= np.radians(self.p.jump_deg))

    def update(self, o):
        """Add one observation -> 'added', 'pending' (held back: jump), 'reset' (track restarted)
        or 'ignored' (too low a confidence)."""
        if o.weight < self.p.min_weight:
            return "ignored"
        self.last_stamp = o.stamp if self.last_stamp is None else max(self.last_stamp, o.stamp)
        if self.centre is None or self._close(o.centre, o.yaw, self.centre, self.yaw):
            self.obs.append(o)
            self.pending.clear()
            self._estimate()
            return "added"
        # a jump: hold it back until enough agreeing ones say the cube really moved
        if self.pending and not self._close(o.centre, o.yaw, self.pending[-1].centre, self.pending[-1].yaw):
            self.pending.clear()
        self.pending.append(o)
        if len(self.pending) >= self.p.confirm:
            self.obs.clear()
            self.obs.extend(self.pending)
            self.pending.clear()
            self.resets += 1
            self._estimate()
            return "reset"
        return "pending"

    def _estimate(self):
        C = np.array([o.centre for o in self.obs])
        w = np.array([o.weight for o in self.obs])
        Y = np.array([o.yaw for o in self.obs])
        self.centre = np.array([_weighted_median(C[:, i], w) for i in range(3)])
        self.yaw, _ = circular_mean_90(Y, w)
        n = len(self.obs)
        d2 = np.sum((C[:, :2] - self.centre[:2]) ** 2, axis=1)
        dy2 = fold_yaw(Y - self.yaw) ** 2
        fs2 = np.array([o.face_spread ** 2 for o in self.obs])
        fy2 = np.array([o.face_spread_yaw ** 2 for o in self.obs])
        # observation scatter, plus the faces' own disagreement (shrinks as views accumulate)
        self.spread = float(np.sqrt(np.average(d2, weights=w) + np.average(fs2, weights=w) / n))
        self.spread_yaw = float(np.sqrt(np.average(dy2, weights=w) + np.average(fy2, weights=w) / n))

    # -- state --------------------------------------------------------------------------
    @property
    def n_obs(self):
        return len(self.obs)

    @property
    def stable(self):
        return (self.n_obs >= self.p.min_obs and not self.pending
                and self.spread <= self.p.stable_mm / 1000 and self.spread_yaw <= np.radians(self.p.stable_deg))

    @property
    def confidence(self):
        """0..1. >= 0.5 exactly when stable; within each half it grows with the number of
        observations, their mean per-frame confidence and a small spread."""
        if self.centre is None:
            return 0.0
        mean_w = float(np.mean([o.weight for o in self.obs]))
        q = min(1.0, self.n_obs / max(self.p.min_obs, 1)) * mean_w \
            * np.exp(-(self.spread * 1000 / max(self.p.stable_mm, 1e-6)) ** 2 / 8)
        q = float(np.clip(q, 0.0, 0.999))
        return 0.5 + 0.5 * q if self.stable else 0.5 * q

    def age(self, now):
        return float("inf") if self.last_stamp is None else now - self.last_stamp


class CubeTracker:
    """CubeTrack per cube id."""

    def __init__(self, params=None):
        self.p = params or TrackerParams()
        self.tracks = {}

    def update(self, cube, o):
        return self.tracks.setdefault(cube, CubeTrack(self.p)).update(o)

    def forget(self, older_than, now):
        """Drop tracks not seen for `older_than` seconds -> the dropped cube ids."""
        gone = [k for k, t in self.tracks.items() if t.age(now) > older_than]
        for k in gone:
            del self.tracks[k]
        return gone
