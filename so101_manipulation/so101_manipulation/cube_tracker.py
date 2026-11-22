"""Where is cube k? One small adapter between the perception topics and the pick/place logic.

    tracker = make_cube_tracker(node, source="auto", ...)
    est = tracker.get("cube_2", since=t_arrived)     # CubeEstimate or None
    est.centre, est.yaw, est.stable, est.spread_mm, est.age_s

Sources (node parameter `cube_source`):

- `tracked`: /cubes/tracked (vision_msgs/Detection3DArray from the cube detector's temporal
  filter, 5 Hz + after every processed frame, one detection per cube seen in the last 30 s).
  header.stamp = time of the last image the cube was seen in; score >= 0.5 exactly when the track
  is stable (>= 5 obs, spread <= 2 mm and <= 3 deg); results[0].pose.covariance[0] =
  spread_xy^2 / 2, [35] = spread_yaw^2. The detector ignores frames while the arm moves (and
  0.3 s after), so a "fresh" estimate needs a stamp after `since`. `TrackedState` additionally
  asks for `min_fresh` new stamps after `since`; if the filter's stable flag still lags behind (old
  far-view observations in its window), `min_count` fresh observations that agree among
  themselves count as stable too, with their own median.
- `detections`: /cubes/detections (one detection per cube per frame), own window per cube:
  robust median of the centre, circular median of the yaw modulo 90 deg, stable once
  `min_count` observations newer than `since` agree within `max_spread_mm` / `max_yaw_spread_deg`.
- `auto` (default): `tracked` for a cube the filter reported in the last 2 s, else `detections`.

All times are the receiving node's monotonic clock (seconds), so `since` can be "the moment the
arm stopped at the view pose"; tracked stamps are converted with the age (ROS now - stamp).
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class CubeEstimate:
    cube_id: str
    centre: np.ndarray        # base_link, m
    yaw: float                # rad, folded to [-45, 45) deg
    stable: bool
    spread_mm: float          # max distance of an observation from the median centre
    yaw_spread_deg: float
    age_s: float              # since the newest observation
    count: int
    score: float = 0.0        # mean faces-seen score


def cube_key(cube_id) -> str:
    """'cube_3', '3', 3, 'K3' -> 'cube_3'."""
    s = str(cube_id).strip()
    if s.lower().startswith("cube_"):
        s = s[5:]
    elif s[:1] in "kK":
        s = s[1:]
    return f"cube_{int(s)}"


def fold_quarter(a: float) -> float:
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def quarter_median(yaws):
    """Median yaw modulo 90 deg: mean direction of 4*yaw, then the median deviation from it."""
    yaws = np.asarray(yaws, float)
    m = math.atan2(np.mean(np.sin(4 * yaws)), np.mean(np.cos(4 * yaws))) / 4
    dev = np.array([fold_quarter(y - m) for y in yaws])
    centre = fold_quarter(m + float(np.median(dev)))
    spread = float(np.max(np.abs([fold_quarter(y - centre) for y in yaws]))) if len(yaws) else 0.0
    return centre, spread


class CubeWindow:
    """Recent observations of one cube; pure Python so it can be tested without ROS."""

    def __init__(self, horizon_s: float = 30.0):
        self.obs = deque()      # (t, centre(3), yaw, score)
        self.horizon_s = horizon_s

    def add(self, t, centre, yaw, score=1.0):
        self.obs.append((float(t), np.asarray(centre, float), float(yaw), float(score)))
        while self.obs and t - self.obs[0][0] > self.horizon_s:
            self.obs.popleft()

    def estimate(self, cube_id, now, since=None, window_s=1.5, min_count=5, max_spread_mm=3.0,
                 max_yaw_spread_deg=4.0, outlier_mm=8.0):
        if not self.obs:
            return None
        newest = self.obs[-1][0]
        lo = newest - window_s
        if since is not None:
            lo = max(lo, since)
        sel = [o for o in self.obs if o[0] >= lo]
        stable_possible = True
        if not sel:
            # nothing new enough: report the last known pose, never stable
            sel = [o for o in self.obs if o[0] >= newest - window_s]
            stable_possible = False
        C = np.array([o[1] for o in sel])
        med = np.median(C, axis=0)
        d = np.linalg.norm(C - med, axis=1) * 1000
        keep = d <= max(outlier_mm, 3 * float(np.median(d)) if len(d) else 0)
        C2 = C[keep] if keep.sum() >= max(1, len(sel) // 2) else C
        yaws = [o[2] for o, k in zip(sel, keep) if k] if keep.sum() >= max(1, len(sel) // 2) else [o[2] for o in sel]
        med = np.median(C2, axis=0)
        spread = float(np.max(np.linalg.norm(C2 - med, axis=1)) * 1000)
        yaw, yspread = quarter_median(yaws)
        n_out = int(len(sel) - len(C2))
        stable = (stable_possible and len(C2) >= min_count and spread <= max_spread_mm
                  and math.degrees(yspread) <= max_yaw_spread_deg and n_out <= len(sel) // 4)
        return CubeEstimate(cube_id=cube_id, centre=med, yaw=yaw, stable=bool(stable), spread_mm=spread,
                            yaw_spread_deg=math.degrees(yspread), age_s=float(now - newest), count=len(C2),
                            score=float(np.mean([o[3] for o in sel])))


class TrackedState:
    """Latest filtered estimate of one cube from /cubes/tracked, plus the fresh stamps seen."""

    def __init__(self):
        self.window = CubeWindow()      # one entry per new image stamp (the filtered centre)
        self.last = None                # (obs_t, centre, yaw, score, spread_xy_mm, spread_yaw_deg)
        self.received = -math.inf       # monotonic time of the last message about this cube

    def add(self, recv_t, obs_t, centre, yaw, score, spread_xy_mm, spread_yaw_deg):
        self.received = recv_t
        new_stamp = self.last is None or obs_t > self.last[0] + 1e-4
        self.last = (obs_t, np.asarray(centre, float), float(yaw), float(score), float(spread_xy_mm), float(spread_yaw_deg))
        if new_stamp:
            self.window.add(obs_t, centre, yaw, score)

    def estimate(self, cube_id, now, since=None, min_fresh=3, window_s=1.5, min_count=5, max_spread_mm=3.0,
                 max_yaw_spread_deg=4.0):
        if self.last is None:
            return None
        obs_t, centre, yaw, score, sxy, syaw = self.last
        fresh = [o for o in self.window.obs if since is None or o[0] >= since]
        own = self.window.estimate(cube_id, now, since, window_s, min_count, max_spread_mm, max_yaw_spread_deg)
        filt_stable = score >= 0.5 and len(fresh) >= min_fresh and (since is None or obs_t >= since)
        if not filt_stable and own is not None and own.stable:
            return own                      # the filter still lags: fresh observations agree
        return CubeEstimate(cube_id=cube_id, centre=centre.copy(), yaw=yaw, stable=bool(filt_stable),
                            spread_mm=sxy, yaw_spread_deg=syaw, age_s=float(now - obs_t), count=len(fresh),
                            score=score)


class CubeTracker:
    """Interface the pick/place node uses."""

    def get(self, cube_id, since=None) -> CubeEstimate | None:
        raise NotImplementedError

    def known(self) -> list:
        raise NotImplementedError

    @staticmethod
    def now() -> float:
        return time.monotonic()


class DetectionsCubeTracker(CubeTracker):
    """CubeTracker on vision_msgs/Detection3DArray (/cubes/detections): own window + median."""

    def __init__(self, node, topic="/cubes/detections", window_s=1.5, min_count=5, max_spread_mm=3.0,
                 max_yaw_spread_deg=4.0, callback_group=None):
        from vision_msgs.msg import Detection3DArray
        self.windows = {}
        self.window_s, self.min_count = window_s, min_count
        self.max_spread_mm, self.max_yaw_spread_deg = max_spread_mm, max_yaw_spread_deg
        self.sub = node.create_subscription(Detection3DArray, topic, self.on_detections, 10,
                                            callback_group=callback_group)

    @staticmethod
    def yaw_of(q) -> float:
        return fold_quarter(2 * math.atan2(q.z, q.w)) if abs(q.x) + abs(q.y) < 1e-3 else \
            fold_quarter(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_detections(self, msg):
        t = self.now()
        for d in msg.detections:
            if d.results:
                cid, score = d.results[0].hypothesis.class_id, d.results[0].hypothesis.score
            else:
                cid, score = d.id, 1.0
            try:
                key = cube_key(cid)
            except ValueError:
                continue
            p = d.bbox.center.position
            self.windows.setdefault(key, CubeWindow()).add(t, [p.x, p.y, p.z], self.yaw_of(d.bbox.center.orientation), score)

    def get(self, cube_id, since=None):
        key = cube_key(cube_id)
        w = self.windows.get(key)
        if w is None:
            return None
        return w.estimate(key, self.now(), since, self.window_s, self.min_count, self.max_spread_mm,
                          self.max_yaw_spread_deg)

    def known(self):
        return sorted(self.windows)


class TrackedCubeTracker(CubeTracker):
    """CubeTracker on /cubes/tracked (the detector's temporal filter)."""

    def __init__(self, node, topic="/cubes/tracked", min_fresh=3, window_s=1.5, min_count=5, max_spread_mm=3.0,
                 max_yaw_spread_deg=4.0, callback_group=None):
        from vision_msgs.msg import Detection3DArray
        self.node = node
        self.states = {}
        self.min_fresh, self.window_s, self.min_count = min_fresh, window_s, min_count
        self.max_spread_mm, self.max_yaw_spread_deg = max_spread_mm, max_yaw_spread_deg
        self.sub = node.create_subscription(Detection3DArray, topic, self.on_tracked, 10, callback_group=callback_group)

    def on_tracked(self, msg):
        recv = self.now()
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        for d in msg.detections:
            if not d.results:
                continue
            r = d.results[0]
            try:
                key = cube_key(r.hypothesis.class_id or d.id)
            except ValueError:
                continue
            stamp = d.header.stamp if (d.header.stamp.sec or d.header.stamp.nanosec) else msg.header.stamp
            age = max(0.0, ros_now - (stamp.sec + stamp.nanosec * 1e-9))
            pose = r.pose.pose
            cov = r.pose.covariance
            sxy = math.sqrt(max(0.0, 2 * cov[0])) * 1000 if len(cov) > 0 else float("nan")
            syaw = math.degrees(math.sqrt(max(0.0, cov[35]))) if len(cov) > 35 else float("nan")
            yaw = fold_quarter(2 * math.atan2(pose.orientation.z, pose.orientation.w))
            self.states.setdefault(key, TrackedState()).add(
                recv, recv - age, [pose.position.x, pose.position.y, pose.position.z], yaw, r.hypothesis.score, sxy, syaw)

    def recent(self, cube_id, within_s=2.0) -> bool:
        st = self.states.get(cube_key(cube_id))
        return st is not None and self.now() - st.received < within_s

    def get(self, cube_id, since=None):
        key = cube_key(cube_id)
        st = self.states.get(key)
        if st is None:
            return None
        return st.estimate(key, self.now(), since, self.min_fresh, self.window_s, self.min_count,
                           self.max_spread_mm, self.max_yaw_spread_deg)

    def known(self):
        return sorted(self.states)


class AutoCubeTracker(CubeTracker):
    """/cubes/tracked when the filter reports the cube, /cubes/detections otherwise."""

    def __init__(self, tracked: TrackedCubeTracker, detections: DetectionsCubeTracker):
        self.tracked, self.detections = tracked, detections

    def get(self, cube_id, since=None):
        if self.tracked.recent(cube_id):
            return self.tracked.get(cube_id, since)
        return self.detections.get(cube_id, since)

    def known(self):
        return sorted(set(self.tracked.known()) | set(self.detections.known()))


class FrameTimes:
    """Capture times (node monotonic clock) of the frames the detector processed, from the per-frame
    /cubes/detections messages (published for every frame taken while the arm stood still, also
    when no cube is in it). "N frames since t without cube k" means cube k is not in view."""

    def __init__(self, maxlen=200):
        self.t = deque(maxlen=maxlen)

    def add(self, obs_t):
        if not self.t or obs_t > self.t[-1] + 1e-4:
            self.t.append(float(obs_t))

    def count_since(self, since) -> int:
        return sum(1 for t in self.t if t >= since)


class FrameClock(FrameTimes):
    def __init__(self, node, topic="/cubes/detections", callback_group=None):
        from vision_msgs.msg import Detection3DArray
        super().__init__()
        self.node = node
        self.sub = node.create_subscription(Detection3DArray, topic, self.on_frame, 10, callback_group=callback_group)

    def on_frame(self, msg):
        recv = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        st = msg.header.stamp
        self.add(recv - max(0.0, ros_now - (st.sec + st.nanosec * 1e-9)))


def make_cube_tracker(node, source="auto", detections_topic="/cubes/detections", tracked_topic="/cubes/tracked",
                      min_fresh=3, **kw) -> CubeTracker:
    if source == "detections":
        return DetectionsCubeTracker(node, detections_topic, **kw)
    if source == "tracked":
        return TrackedCubeTracker(node, tracked_topic, min_fresh, **kw)
    if source == "auto":
        return AutoCubeTracker(TrackedCubeTracker(node, tracked_topic, min_fresh, **kw),
                               DetectionsCubeTracker(node, detections_topic, **kw))
    raise ValueError(f"unknown cube_source '{source}' (auto, tracked, detections)")
