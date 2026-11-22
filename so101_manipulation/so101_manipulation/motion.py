"""Joint trajectories straight to the controllers (arm_controller, gripper_controller), with
state and lag monitoring. Used from a blocking action thread while a MultiThreadedExecutor
spins the subscriptions in other threads.

Every arm goal is checked against the planning limits and |wrist_roll| <= roll_limit before it
is sent; nothing outside those ever reaches the controller from here.
"""

from __future__ import annotations

import math

import numpy as np
import threading
import time
from collections import deque
from dataclasses import dataclass

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import ARM, GRIPPER, JOINT_LIMITS, within_limits
from .soft_grasp import clamp_gripper_points


class Canceled(Exception):
    """The pick/place goal was canceled: motion was stopped where it was."""


class MotionError(Exception):
    pass


@dataclass
class StateSample:
    t: float
    ref: dict
    fb: dict


@dataclass
class MotionResult:
    ok: bool
    stopped: bool = False     # stopped by the monitor (contact)
    error: str = ""


def duration(secs: float) -> Duration:
    return Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))


def clamp_arm(q, roll_limit=math.pi / 2):
    """A biased arm command kept inside the planning limits and the wrist_roll limit: a trim at a
    joint limit corrects up to the limit instead of making the goal invalid (wrist_flex 95.2 deg
    was refused at a far, tilted grasp, 2026-11-22)."""
    out = np.array(q, float)
    for k, name in enumerate(ARM):
        lo, hi = JOINT_LIMITS[name]
        if name == "wrist_roll":
            lo, hi = max(lo, -roll_limit), min(hi, roll_limit)
        out[k] = min(max(out[k], lo), hi)
    return out


class Motion:
    def __init__(self, node, callback_group, min_move_s=0.8, roll_limit=math.pi / 2, settle_s=0.3):
        self.node = node
        self.min_move_s, self.roll_limit, self.settle_s = min_move_s, roll_limit, settle_s
        self.lock = threading.Lock()
        self.js, self.js_t = {}, 0.0
        self.samples = {"arm": deque(maxlen=500), "gripper": deque(maxlen=500)}
        self.load = {}
        self.grip_current = deque(maxlen=400)   # (t, A) of the gripper servo
        self.grip_torque_limit = None           # gripper torque_limit state (0.1 % units), if exported
        self.cancel_check = lambda: False
        self.gripper_floor = None  # rad; every gripper goal is clamped to >= this (see soft_grasp.py)
        self.active = {}        # controller -> client goal handle
        cg = callback_group
        node.create_subscription(JointState, "/joint_states", self.on_js, 20, callback_group=cg)
        node.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state",
                                 lambda m: self.on_state("arm", m), 20, callback_group=cg)
        node.create_subscription(JointTrajectoryControllerState, "/gripper_controller/controller_state",
                                 lambda m: self.on_state("gripper", m), 20, callback_group=cg)
        node.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_dyn, 10, callback_group=cg)
        self.clients = {
            "arm": ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory", callback_group=cg),
            "gripper": ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory",
                                    callback_group=cg),
        }
        self.joints = {"arm": ARM, "gripper": [GRIPPER]}

    # --- state ----------------------------------------------------------------------------

    def on_js(self, m):
        with self.lock:
            self.js.update(zip(m.name, m.position))
            self.js_t = time.monotonic()

    def on_state(self, ctrl, m):
        names = list(m.joint_names)
        if not m.reference.positions or not m.feedback.positions:
            return
        s = StateSample(time.monotonic(), dict(zip(names, m.reference.positions)), dict(zip(names, m.feedback.positions)))
        with self.lock:
            self.samples[ctrl].append(s)

    def on_dyn(self, m):
        with self.lock:
            for name, iv in zip(m.joint_names, m.interface_values):
                vals = dict(zip(iv.interface_names, iv.values))
                if "load" in vals:
                    self.load[name] = float(vals["load"])
                if name == GRIPPER:
                    if "current" in vals:
                        self.grip_current.append((time.monotonic(), float(vals["current"])))
                    if "torque_limit" in vals:
                        self.grip_torque_limit = float(vals["torque_limit"])

    def ready(self, timeout=5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                ok = all(j in self.js for j in ARM + [GRIPPER])
            if ok and all(c.server_is_ready() for c in self.clients.values()):
                return True
            time.sleep(0.05)
        return False

    def q_arm(self):
        with self.lock:
            return [self.js[n] for n in ARM]

    def grip(self) -> float:
        with self.lock:
            return self.js[GRIPPER]

    def gripper_load(self) -> float:
        with self.lock:
            return self.load.get(GRIPPER, float("nan"))

    def gripper_current_ma(self, window_s=0.3) -> float:
        """Mean gripper current over the last window_s [mA]; NaN if the hardware reports none."""
        now = time.monotonic()
        with self.lock:
            v = [c for t, c in self.grip_current if now - t <= window_s and c == c]
        return 1000.0 * sum(v) / len(v) if v else float("nan")

    def gripper_torque_limit(self):
        with self.lock:
            return self.grip_torque_limit

    def samples_since(self, ctrl, t0):
        with self.lock:
            return [s for s in self.samples[ctrl] if s.t > t0]

    def mean_lag(self, joint="shoulder_lift", n=15, timeout=2.0) -> float:
        """Mean (reference - feedback) of `joint` over the next n arm controller states [deg]."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            got = self.samples_since("arm", t0)
            if len(got) >= n:
                return sum(math.degrees(s.ref[joint] - s.fb[joint]) for s in got[:n]) / n
            self.check_cancel()
            time.sleep(0.01)
        got = self.samples_since("arm", t0)
        if not got:
            raise MotionError("no /arm_controller/controller_state")
        return sum(math.degrees(s.ref[joint] - s.fb[joint]) for s in got) / len(got)

    # --- goals ----------------------------------------------------------------------------

    def check_cancel(self):
        if self.cancel_check():
            self.stop_all()
            raise Canceled()

    def validate(self, ctrl, points):
        for p in points:
            if ctrl == "arm":
                why = within_limits(p, self.roll_limit)
                if why:
                    raise MotionError(f"refusing arm goal: {why}")
            else:
                lo, hi = JOINT_LIMITS[GRIPPER]
                if not lo - 1e-6 <= p[0] <= hi + 1e-6:
                    raise MotionError(f"refusing gripper goal {math.degrees(p[0]):.1f} deg")

    def timed(self, ctrl, points, speed):
        """time_from_start per point: the largest joint change at `speed` rad/s, >= min_move_s."""
        with self.lock:
            prev = [self.js[n] for n in self.joints[ctrl]]
        t, out = 0.0, []
        for p in points:
            t += max(self.min_move_s, max(abs(a - b) for a, b in zip(p, prev)) / max(1e-3, speed))
            out.append(t)
            prev = p
        return out

    def send_async(self, ctrl, points, times, through=False):
        """`through`: no velocities on the inner points, so the controller passes them without stopping
        (the last point still ends at rest)."""
        if ctrl == "gripper":
            # HARD RULE: no gripper command below the cube width (minus a small squeeze)
            points, clamped = clamp_gripper_points(points, self.gripper_floor)
            if clamped:
                self.node.get_logger().warn(
                    f"gripper goal clamped to the floor {math.degrees(self.gripper_floor):.1f} deg")
        self.validate(ctrl, points)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(self.joints[ctrl])
        for k, (p, t) in enumerate(zip(points, times)):
            last = k == len(points) - 1
            pt = JointTrajectoryPoint(positions=[float(v) for v in p],
                                      velocities=[0.0] * len(p) if (last or not through) else [])
            pt.time_from_start = duration(t)
            g.trajectory.points.append(pt)
        client = self.clients[ctrl]
        if not client.wait_for_server(timeout_sec=5.0):
            raise MotionError(f"{ctrl} controller action server not available")
        fut = client.send_goal_async(g)
        self.wait(fut, 5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise MotionError(f"{ctrl} goal rejected")
        self.active[ctrl] = gh
        return gh

    def wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                raise MotionError("timeout waiting for the controller")
            time.sleep(0.005)

    def trim_arm(self, target, tol_deg=0.3, max_iter=3, max_bias_deg=5.0, move_s=0.8):
        """Free space only: bring the MEASURED arm joints to `target` by adding the static tracking
        error to the command (at servo P 16 the elbow stays ~2.3 deg short under gravity, which put
        the jaws 8 mm short and 7 mm low over a cube, 2026-11-22). Returns the bias (rad per joint)
        to add to nearby commands, e.g. the descent; never call it in contact."""
        target = np.asarray(target, float)
        bias = np.zeros(len(ARM))
        lim = math.radians(max_bias_deg)
        for _ in range(max_iter):
            time.sleep(self.settle_s)
            err = target - np.asarray(self.q_arm())
            if np.max(np.abs(err)) < math.radians(tol_deg):
                break
            bias = np.clip(bias + err, -lim, lim)
            r = self.run("arm", [clamp_arm(target + bias, self.roll_limit)], times=[move_s])
            if not r.ok:
                break
        return bias

    def run(self, ctrl, points, speed=None, times=None, monitor=None, settle=True, bias=None,
            through=False) -> MotionResult:
        """Send and wait; `monitor(samples)` -> True stops the motion (goal canceled: the controller
        holds where the joints are). Raises Canceled when the action goal is canceled. `bias` (arm
        only, from trim_arm) is added to every point."""
        self.check_cancel()
        if bias is not None and ctrl == "arm":
            points = [clamp_arm(np.asarray(p, float) + bias, self.roll_limit) for p in points]
        if times is None:
            times = self.timed(ctrl, points, speed)
        t0 = time.monotonic()
        gh = self.send_async(ctrl, points, times, through=through)
        res_fut = gh.get_result_async()
        last = t0
        deadline = t0 + times[-1] + 10.0
        while not res_fut.done():
            if self.cancel_check():
                self.stop_all()
                raise Canceled()
            if monitor is not None:
                new = self.samples_since("arm" if ctrl == "arm" else "gripper", last)
                if new:
                    last = new[-1].t
                    if monitor(new):
                        self.stop(ctrl)
                        return MotionResult(ok=False, stopped=True)
            if time.monotonic() > deadline:
                self.stop(ctrl)
                return MotionResult(ok=False, error="trajectory did not finish in time")
            time.sleep(0.01)
        self.active.pop(ctrl, None)
        r = res_fut.result().result
        if settle and self.settle_s > 0:
            time.sleep(self.settle_s)
        if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return MotionResult(ok=False, error=f"{ctrl}: {r.error_string or r.error_code}")
        return MotionResult(ok=True)

    def stop(self, ctrl):
        gh = self.active.pop(ctrl, None)
        if gh is not None:
            fut = gh.cancel_goal_async()
            try:
                self.wait(fut, 2.0)
            except MotionError:
                pass

    def stop_all(self):
        for c in list(self.active):
            self.stop(c)
