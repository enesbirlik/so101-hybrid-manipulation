#!/usr/bin/env python3
"""Cube pick and place action server for the SO-101.

  /pick_cube   so101_manipulation_interfaces/action/PickCube    search, approach, refine, descend, grasp, verify, lift
  /place_cube  so101_manipulation_interfaces/action/PlaceCube   carry, refine (stacking), descend, release, retreat
  ~/open_gripper  std_srvs/Trigger   open the gripper slowly where the arm is (the only "let go" besides PlaceCube)

Motion goes straight to arm_controller / gripper_controller as joint-space trajectories, each
segment checked against the table model (every link's mesh hull, the wrist camera and a carried
cube) before it is sent. See README.md for the parameters, the phases and the safety notes.
"""

from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_manipulation_interfaces.action import PickCube, PlaceCube

from . import soft_grasp as sg
from .cube_tracker import cube_key, make_cube_tracker
from .kinematics import ARM
from .grasp import WEIGHT_TILTED, GraspPlanner, default_views, load_hand_eye
from .hooks import GripperTorqueLimitHook
from .kinematics import ArmModel, perception_config_dir
from .motion import Canceled, Motion, MotionError

NAN = float("nan")

# name: (default, description). Lengths in m unless _mm, angles in deg, speeds in rad/s on the
# joint that moves furthest unless _deg_s.
PARAMS = {
    # perception
    "cube_source": ("auto", "cube adapter: 'tracked' (/cubes/tracked, the detector's filter), 'detections' "
                            "(/cubes/detections + own median window), 'auto' (tracked when it reports the cube)"),
    "tracked_topic": ("/cubes/tracked", "filtered cubes (vision_msgs/Detection3DArray, score >= 0.5 = stable)"),
    "detections_topic": ("/cubes/detections", "per-frame cubes (vision_msgs/Detection3DArray)"),
    "obs_min_fresh": (3, "tracked: new image stamps after the arm stopped needed for a fresh stable estimate"),
    "cube_edge_mm": (30.0, "cube edge"),
    "obs_window_s": (1.5, "observations older than this (s) are not used for an estimate"),
    "obs_min_count": (5, "observations needed for a stable estimate"),
    "obs_max_spread_mm": (3.0, "stable: every observation within this of the median centre"),
    "obs_max_yaw_spread_deg": (4.0, "stable: every yaw (mod 90) within this of the median"),
    "obs_settle_s": (0.4, "after the arm stops, wait this long before observations count"),
    # search
    "search_pans_deg": ([0.0, -25.0, 25.0, -45.0, 45.0], "pan angles of the search views"),
    "search_initial_wait_s": (1.0, "first look for the cube where the arm is, this long"),
    "search_view_wait_s": (2.0, "wait for a stable cube at each view pose"),
    "search_gripper_deg": (12.0, "jaws nearly shut while searching (they hide less of the image); clamped to the floor"),
    # grasp geometry
    "open_deg": (30.0, "gripper opening for the approach (~53 mm gap)"),
    "grasp_tip_height": (0.011, "fixed jaw tip above the cube's bottom at the grasp"),
    "hover_height": (0.07, "fixed jaw tip above the cube's bottom at the hover"),
    "hover_min_height": (0.05, "lowest hover tried when the hover is out of reach"),
    "descend_step": (0.01, "height step of the descent"),
    "gap_fraction": (0.5, "where the cube centre goes between the fixed (0) and moving (1) jaw tip; "
                          "0.5 = gap midpoint"),
    "allow_tilt": (True, "allow a tilted approach (<= max_tilt_deg) beyond vertical reach"),
    "max_tilt_deg": (15.0, "largest jaw tilt of the fallback"),
    "vertical_tol_deg": (3.0, "jaws count as vertical within this"),
    "ik_tolerance_mm": (2.0, "largest IK position error accepted"),
    "roll_limit_deg": (90.0, "|wrist_roll| never beyond this (camera cable)"),
    "min_final_clearance_mm": (3.0, "lowest point of the hand above the table at the grasp pose"),
    # close-up refinement
    "refine_max_iter": (2, "close-up refinements at the hover"),
    "trim_tol_deg": (0.3, "hover trim: stop when every measured arm joint is within this of the plan"),
    "trim_max_iter": (3, "hover trim: correction moves at most"),
    "trim_max_bias_deg": (5.0, "hover trim: largest correction added to a joint command"),
    "refine_min_shift_mm": (2.0, "re-target when the close-up estimate moved more than this"),
    "refine_max_shift_mm": (40.0, "abort when the close-up estimate moved more than this"),
    "refine_timeout_s": (3.0, "wait this long for a stable close-up estimate"),
    "refine_required": (False, "abort when the close-up sees no stable cube (else keep the far estimate)"),
    # contact on the way down (arm)
    "contact_joint": ("shoulder_lift", "joint whose lag (reference - feedback) shows a contact"),
    "descend_contact_lag_deg": (2.5, "lag above the hover baseline during the descent = contact"),
    "descend_lag_samples": (15, "controller states averaged after each step"),
    # soft grasp (gripper)
    "close_speed_deg_s": (6.0, "closing ramp speed"),
    "open_speed_deg_s": (20.0, "opening speed (release, abort)"),
    "contact_lag_deg": (1.0, "contact when measured - commanded (= -(controller lag), positive when blocked) is this "
                                "far above its free-closing baseline"),
    "contact_samples": (3, "consecutive samples over the threshold"),
    "contact_load_pct": (0.0, "|load - baseline| that also counts as contact; 0 = off"),
    "squeeze_deg": (1.5, "hold target = contact angle - this, never below the floor"),
    "squeeze_mm": (1.5, "HARD FLOOR: no gripper command below the angle of a gap of cube_edge_mm - this "
                        "(30 mm -> ~11.5 deg); the closing ramp ends there"),
    "contact_window_deg": (5.0, "contact must lie within expected +- this, else abort"),
    "gap_zero_mm": (13.0, "jaw gap at 0 deg"),
    "gap_per_deg_mm": (1.35, "jaw gap per degree (<= 30 deg)"),
    "empty_deadband_deg": (0.56, "empty jaws stop this far above the command (servo deadband; real arm: "
                                 "command 11.48 -> 12.04 deg)"),
    "held_angle_margin_deg": (0.25, "held: measured > command + empty_deadband_deg + this"),
    "empty_angle_tol_deg": (0.15, "empty: measured <= command + empty_deadband_deg + this (and low current)"),
    "held_current_ma": (12.0, "held: or the gripper current at/above this (real arm: 13-20 mA with the cube, 6 empty)"),
    "empty_current_ma": (10.0, "empty: current below this"),
    "current_window_s": (0.3, "the current is averaged over this"),
    "grip_settle_s": (0.4, "wait after the close before judging the grip"),
    "hold_ramp_s": (0.15, "time to move the command to the hold target"),
    "grasp_retries": (1, "retries after a contact outside the window"),
    "sim_contact_deg": (NAN, "MOCK ONLY: pretend a cube blocks the jaws at this angle (NaN = off); "
                             "refused unless robot_description uses mock_components"),
    "gripper_torque_limit": (200.0, "gripper torque limit during the grasp, 0.1 % units (200 = 20 %, holds a "
                                    "paper cube); <= 0 = do not touch it"),
    "gripper_torque_restore": (1000.0, "torque limit restored after the gripper opened (clamped by the hardware)"),
    "torque_limit_topic": ("/gripper_torque_controller/commands", "std_msgs/Float64MultiArray command topic"),
    "torque_limit_timeout_s": (3.0, "abort when the torque_limit readback does not match within this"),
    "torque_limit_required": (True, "abort when the torque limit interface is missing (skipped on mock hardware)"),
    # place
    "release_gap": (0.002, "release this much above the surface (contact stops earlier)"),
    "stack_max_age_s": (60.0, "the cube to stack on must have been seen this recently"),
    # motion / safety
    "transit_speed": (0.9, "rad/s, transit"),
    "approach_speed": (0.45, "rad/s, re-targeting at the hover"),
    "descend_speed": (0.2, "rad/s, descent steps"),
    "lift_speed": (0.3, "rad/s, lift / retreat"),
    "min_move_s": (0.8, "shortest move"),
    "settle_s": (0.3, "pause after each move"),
    "transit_clearance_mm": (45.0, "lowest point (arm, camera, carried cube) above the table on transits; "
                                   "cubes are 30 mm tall"),
    "via_height": (0.12, "tip height of the via pose above a target"),
    "table_z": (NAN, "table surface in base_link; NaN = so101_kinematics.yaml"),
}


class Fail(Exception):
    def __init__(self, phase, message):
        super().__init__(message)
        self.phase, self.message = phase, message


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("cube_pick_place")
        self.p = SimpleNamespace()
        for name, (default, desc) in PARAMS.items():
            setattr(self.p, name, self.declare_parameter(name, default, ParameterDescriptor(description=desc)).value)
        p = self.p
        self.arm_bias = None   # static servo error measured at the hover (trim_arm), reused for the descent
        self.cg = ReentrantCallbackGroup()
        self.model = ArmModel(table_z=p.table_z)
        self.roll_limit = math.radians(p.roll_limit_deg)
        self.planner = GraspPlanner(self.model, roll_limit=self.roll_limit, ik_tol_mm=p.ik_tolerance_mm,
                                    vertical_tol_deg=p.vertical_tol_deg, max_tilt_deg=p.max_tilt_deg,
                                    allow_tilt=p.allow_tilt, min_clearance=p.min_final_clearance_mm / 1000)
        self.planner.roll_map()
        try:
            T_gc = load_hand_eye(perception_config_dir() / "wrist_camera_hand_eye.yaml")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no hand-eye file ({e}): only the near search views")
            T_gc = None
        self.T_gc = T_gc
        self.views = default_views(self.model, T_gc, list(p.search_pans_deg))
        self.tracker = make_cube_tracker(self, p.cube_source, p.detections_topic, p.tracked_topic,
                                         int(p.obs_min_fresh), window_s=p.obs_window_s,
                                         min_count=int(p.obs_min_count), max_spread_mm=p.obs_max_spread_mm,
                                         max_yaw_spread_deg=p.obs_max_yaw_spread_deg, callback_group=self.cg)
        self.motion = Motion(self, self.cg, p.min_move_s, self.roll_limit, p.settle_s)
        self.sg = sg.SoftGraspParams(
            closing_speed_deg_s=p.close_speed_deg_s, lag_deg=p.contact_lag_deg, samples=int(p.contact_samples),
            load_pct=p.contact_load_pct, squeeze_deg=p.squeeze_deg, window_deg=p.contact_window_deg,
            empty_deadband_deg=p.empty_deadband_deg, held_angle_margin_deg=p.held_angle_margin_deg,
            empty_angle_tol_deg=p.empty_angle_tol_deg, held_current_ma=p.held_current_ma, empty_current_ma=p.empty_current_ma,
            edge_mm=p.cube_edge_mm, squeeze_mm=p.squeeze_mm, gap_zero_mm=p.gap_zero_mm, gap_per_deg_mm=p.gap_per_deg_mm)
        self.floor_deg = sg.floor_deg(self.sg)
        # HARD RULE: every gripper goal this node sends is clamped to >= the floor (motion.py)
        self.motion.gripper_floor = math.radians(self.floor_deg)
        self.hook = GripperTorqueLimitHook(self, self.motion, p.gripper_torque_limit, p.gripper_torque_restore,
                                           p.torque_limit_topic, p.torque_limit_timeout_s, p.torque_limit_required)
        self.expected_contact = sg.expected_contact_deg(p.cube_edge_mm, p.gap_zero_mm, p.gap_per_deg_mm)
        self.edge = p.cube_edge_mm / 1000
        self.busy = threading.Lock()
        self.holding = None          # dict once a cube is held
        self.is_mock = None
        self.create_subscription(String, "/robot_description", self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=self.cg)
        self.goal = None
        self.pick_server = ActionServer(self, PickCube, "pick_cube", execute_callback=self.execute_pick,
                                        goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                        callback_group=self.cg)
        self.place_server = ActionServer(self, PlaceCube, "place_cube", execute_callback=self.execute_place,
                                         goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                         callback_group=self.cg)
        self.create_service(Trigger, "~/open_gripper", self.on_open_gripper, callback_group=self.cg)
        sim = "" if math.isnan(p.sim_contact_deg) else f", SIMULATED contact at {p.sim_contact_deg:.1f} deg (mock only)"
        self.get_logger().info(
            f"ready: /pick_cube, /place_cube; table_z {self.model.table_z * 1000:.1f} mm, expected contact "
            f"{self.expected_contact:.1f} deg for a {p.cube_edge_mm:.0f} mm cube, gripper floor {self.floor_deg:.1f} deg, "
            f"{len(self.views)} search views{sim}")

    # --- plumbing -----------------------------------------------------------------------------

    def on_description(self, msg):
        self.is_mock = "mock_components/GenericSystem" in msg.data

    def on_goal(self, _goal):
        if self.busy.locked():
            self.get_logger().warn("goal rejected: another pick/place is running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _gh):
        return CancelResponse.ACCEPT

    def feedback(self, phase, detail=""):
        self.get_logger().info(f"[{phase}] {detail}")
        if self.goal is None:
            return
        fb = self.goal.feedback_cls()
        fb.phase, fb.detail = phase, detail
        if hasattr(fb, "gripper_deg"):
            try:
                fb.gripper_deg = float(self.grip_measured_deg())
            except Exception:  # noqa: BLE001
                pass
        self.goal.handle.publish_feedback(fb)

    def begin(self, gh, feedback_cls):
        self.busy.acquire()
        self.goal = SimpleNamespace(handle=gh, feedback_cls=feedback_cls)
        self.motion.cancel_check = lambda: gh.is_cancel_requested

    def end(self):
        self.motion.cancel_check = lambda: False
        self.goal = None
        self.busy.release()

    def check_preconditions(self):
        if not self.motion.ready(5.0):
            raise Fail("setup", "no /joint_states or controller action servers (is the control stack running?)")
        if not math.isnan(self.p.sim_contact_deg) and self.is_mock is not True:
            raise Fail("setup", "sim_contact_deg is set but robot_description is not mock hardware: refusing")

    # --- gripper state (with the mock-only simulated cube) ------------------------------------

    def grip_sample(self):
        """(commanded deg, measured deg) of the gripper, measured with the simulated cube if enabled."""
        got = self.motion.samples_since("gripper", time.monotonic() - 0.5)
        if not got:
            g = math.degrees(self.motion.grip())
            return g, sg.simulated_measurement(g, g, self.p.sim_contact_deg)
        s = got[-1]
        ref, fb = math.degrees(s.ref["gripper"]), math.degrees(s.fb["gripper"])
        return ref, sg.simulated_measurement(ref, fb, self.p.sim_contact_deg)

    def grip_measured_deg(self):
        return self.grip_sample()[1]

    def gripper_to(self, deg, speed_deg_s):
        cur = math.degrees(self.motion.grip())
        dur = max(0.3, abs(cur - deg) / max(1.0, speed_deg_s))
        r = self.motion.run("gripper", [[math.radians(deg)]], times=[dur])
        if not r.ok:
            self.get_logger().warn(f"gripper move: {r.error}")

    # --- transit with clearance checks --------------------------------------------------------

    def raise_pose(self, q):
        up = np.array(q, float)
        up[1] = min(up[1], 0.0)
        up[2] = min(up[2], 0.3)
        up[4] = float(np.clip(up[4], -self.roll_limit, self.roll_limit))
        return up

    def above_pose(self, q_goal, grip):
        """Tip `via_height` above the goal's tip, jaws as vertical as reachable, same roll."""
        tip = self.model.tip(q_goal)
        off = self.model.gap_offset(q_goal, grip, self.p.gap_fraction)
        r = self.planner.solve((tip + off)[:2], self.p.via_height, None, grip, self.p.gap_fraction, WEIGHT_TILTED,
                               roll_hint=q_goal[4], roll_weight=0.5)
        return r.q if r.pos_err_mm < 10 else None

    def plan_path(self, q_goal, grip, load_below=0.0, with_above=True):
        """List of arm poses from here to q_goal whose straight joint moves keep the clearance."""
        need = self.p.transit_clearance_mm / 1000
        qa = np.array(self.motion.q_arm())
        q_goal = np.array(q_goal, float)
        start_low = self.model.lowest_with_load(qa, grip, load_below)
        goal_low = self.model.lowest_with_load(q_goal, grip, load_below)
        vias = {"raise": self.raise_pose(qa)}
        if with_above:
            a = self.above_pose(q_goal, grip)
            if a is not None:
                vias["above"] = a
        cands = [[], ["above"], ["raise", "above"], ["raise"]]
        tried = []
        for c in cands:
            if any(v not in vias for v in c):
                continue
            path = [vias[v] for v in c] + [q_goal]
            pts = [qa] + path
            worst, ok = [], True
            for i in range(len(pts) - 1):
                req = need
                if i == 0:
                    req = min(req, start_low - 0.002)      # leaving a low pose: just do not go lower
                if i == len(pts) - 2:
                    req = min(req, goal_low - 0.002)       # arriving at a low pose
                clr = self.model.path_clearance(pts[i], pts[i + 1], grip, load_below, n=25)
                worst.append(clr)
                if clr < req:
                    ok = False
                    break
            tried.append(f"{'+'.join(c) or 'direct'}: " + ", ".join(f"{w * 1000:.0f}" for w in worst) + " mm")
            if ok:
                return path
        raise Fail("transit", "no path with enough clearance (" + "; ".join(tried) + ")")

    def transit(self, q_goal, grip_deg, phase, load_below=0.0, speed=None, with_above=True):
        path = self.plan_path(q_goal, math.radians(grip_deg), load_below, with_above)
        r = self.motion.run("arm", path, speed or self.p.transit_speed)
        if not r.ok:
            raise Fail(phase, f"arm move failed: {r.error}")

    # --- perception helpers -------------------------------------------------------------------

    def wait_stable(self, cube, since, timeout):
        end = time.monotonic() + timeout
        est = None
        while time.monotonic() < end:
            self.motion.check_cancel()
            est = self.tracker.get(cube, since=since)
            if est is not None and est.stable:
                return est
            time.sleep(0.05)
        return None

    def surface_base(self, est):
        """Height of the cube's bottom above the table (0 on the table, ~edge on another cube)."""
        b = float(est.centre[2] - self.edge / 2 - self.model.table_z)
        return b if b > 0.008 else 0.0

    def search(self, cube):
        self.feedback("search", f"looking for {cube}")
        est = self.wait_stable(cube, time.monotonic(), self.p.search_initial_wait_s)
        if est is not None:
            self.feedback("search", f"{cube} seen from here")
            return est
        hint = self.tracker.get(cube)
        views = list(self.views)
        if hint is not None and self.T_gc is not None:
            # look first where the cube was last seen: views sorted by the angle between the
            # camera's optical axis and the remembered position
            def off_axis(v):
                T = self.model.hand(v[1]) @ self.T_gc
                d = hint.centre - T[:3, 3]
                return math.acos(float(np.clip(T[:3, 2] @ d / np.linalg.norm(d), -1, 1)))
            views.sort(key=off_axis)
        if self.holding is None and abs(math.degrees(self.motion.grip()) - self.p.search_gripper_deg) > 2:
            self.gripper_to(self.p.search_gripper_deg, self.p.open_speed_deg_s)
        grip = math.degrees(self.motion.grip())
        for name, q in views:
            self.feedback("search", f"view {name}")
            try:
                self.transit(q, grip, "search", with_above=False)
            except Fail as e:
                self.get_logger().warn(f"view {name} skipped: {e.message}")
                continue
            est = self.wait_stable(cube, time.monotonic() + self.p.obs_settle_s, self.p.search_view_wait_s)
            if est is not None:
                self.feedback("search", f"{cube} found from view {name}: ({est.centre[0] * 100:.1f}, "
                                        f"{est.centre[1] * 100:.1f}) cm, yaw {math.degrees(est.yaw):+.0f} deg")
                return est
        raise Fail("search", f"{cube} not found from {len(views)} view poses")

    # --- pick ---------------------------------------------------------------------------------

    def plan_grasp(self, est, prefer_roll=None):
        base = self.surface_base(est)
        p = self.p
        plan, msg = self.planner.plan_cube(est.centre[:2], est.yaw, math.radians(p.open_deg), p.hover_height + base,
                                           p.hover_min_height + base, p.grasp_tip_height + base, p.descend_step,
                                           p.gap_fraction, prefer_roll=prefer_roll)
        if plan is None:
            raise Fail("approach", msg)
        return plan

    def refine(self, cube, est, plan):
        """Close-up re-detection at the hover. Any face of the cube counts (the 2026-11-21 scripts
        skipped the refinement when the close-up saw another part of the object). Returns
        (est, plan, total shift mm)."""
        p = self.p
        total = 0.0
        for k in range(int(p.refine_max_iter)):
            since = time.monotonic() + p.obs_settle_s
            new = self.wait_stable(cube, since, p.refine_timeout_s + p.obs_settle_s)
            if new is None:
                if p.refine_required:
                    raise Fail("refine", f"no stable close-up view of {cube}")
                self.feedback("refine", "no stable close-up estimate: keeping the far one")
                break
            shift = float(np.linalg.norm(new.centre[:2] - est.centre[:2]) * 1000)
            dyaw = math.degrees(sg_fold(new.yaw - est.yaw))
            self.feedback("refine", f"close-up {k + 1}: ({new.centre[0] * 100:.1f}, {new.centre[1] * 100:.1f}) cm, "
                                    f"shift {shift:.1f} mm, yaw {dyaw:+.1f} deg")
            if shift > p.refine_max_shift_mm:
                raise Fail("refine", f"close-up estimate {shift:.0f} mm from the far one (max {p.refine_max_shift_mm:.0f}): "
                                     "wrong cube or bad calibration")
            if shift <= p.refine_min_shift_mm and abs(dyaw) < 3.0:
                est = new
                break
            plan = self.plan_grasp(new, prefer_roll=plan.roll)
            total += shift
            est = new
            r = self.motion.run("arm", [plan.hover.q], p.approach_speed)
            if not r.ok:
                raise Fail("refine", f"re-target move failed: {r.error}")
        return est, plan, total

    def descend(self, plan, grip_deg, stop_on_contact_ok=False):
        """One continuous move down through plan.descent with a contact stop. Returns the index of the
        descent point nearest to where the hand ended; raises on contact unless stop_on_contact_ok
        (place: contact = the cube is down)."""
        p = self.p
        j = p.contact_joint
        # at the hover, in free space: take out the servos' static error, and reuse it below
        self.arm_bias = self.motion.trim_arm(plan.hover.q, p.trim_tol_deg, int(p.trim_max_iter), p.trim_max_bias_deg)
        self.feedback("descend", "hover trim " + ", ".join(f"{n.split('_')[1]} {math.degrees(b):+.1f}"
                                                         for n, b in zip(ARM, self.arm_bias) if abs(b) > 1e-4))
        base = self.motion.mean_lag(j, int(p.descend_lag_samples))
        # Gravity alone drifts the lag slowly as the arm goes down (+1.1 deg over 6 cm at servo P 32,
        # 2026-11-22); a jaw landing on something lifts it at once. One move instead of 1 cm steps
        # (stop, settle, measure: ~8 s and stick-slip at every start).
        peak = [0.0]

        def monitor(samples):
            for s_ in samples:
                ex = math.degrees(s_.ref[j] - s_.fb[j]) - base
                peak[0] = max(peak[0], ex)
                if ex > p.descend_contact_lag_deg:
                    return True
            return False
        pts = [r.q for r in plan.descent]
        span = float(np.max(np.abs(np.asarray(pts[-1]) - np.asarray(plan.hover.q))))
        total = max(self.motion.min_move_s, span / max(1e-3, p.descend_speed))
        times = [total * (k + 1) / len(pts) for k in range(len(pts))]
        self.feedback("descend", f"down to {plan.heights[-1] * 100:.1f} cm in {total:.1f} s")
        res = self.motion.run("arm", pts, times=times, monitor=monitor, bias=self.arm_bias, through=True)
        h_now = float(self.model.tip(np.asarray(self.motion.q_arm()))[2] - self.model.table_z)
        i = int(np.argmin([abs(h - h_now) for h in plan.heights]))
        if res.stopped:
            msg = f"contact at {h_now * 100:.1f} cm ({j} lag {peak[0]:+.1f} deg over the hover)"
            if stop_on_contact_ok:
                self.feedback("descend", msg + ": stopping here")
                return i
            self.feedback("descend", msg + ": backing up")
            self.motion.run("arm", [plan.hover.q], p.lift_speed, bias=self.arm_bias)
            raise Fail("descend", f"something under the jaws: {msg}")
        if not res.ok:
            raise Fail("descend", f"arm move failed: {res.error}")
        self.feedback("descend", f"down at {h_now * 100:.1f} cm, {j} lag peak {peak[0]:+.2f} deg over the hover")
        return len(plan.descent) - 1

    def soft_close(self):
        """Close slowly down to the floor (never lower), stop early on a contact, then judge the grip.

        Returns (state, hold command deg, contact event or None, why) with state one of
        sg.HELD / sg.EMPTY / sg.UNCERTAIN, or "early" for a contact above the window (something
        wider than the cube between the jaws; the jaws are frozen there)."""
        p, P = self.p, self.sg
        # static baseline: the gripper standing open
        t0 = time.monotonic()
        time.sleep(0.3)
        lags = []
        for s in self.motion.samples_since("gripper", t0):
            ref, fb = math.degrees(s.ref["gripper"]), math.degrees(s.fb["gripper"])
            lags.append(sg.simulated_measurement(ref, fb, p.sim_contact_deg) - ref)
        load0 = self.motion.gripper_load()
        det = sg.ContactDetector(P, static_baseline_lag=float(np.median(lags)) if lags else 0.0,
                                 static_baseline_load=0.0 if math.isnan(load0) else load0,
                                 ignore_above_deg=self.expected_contact + P.window_deg)
        start = self.grip_measured_deg()
        end = sg.ramp_end_deg(self.expected_contact, P)          # = the floor
        dur = max(0.5, (start - end) / P.closing_speed_deg_s)
        self.feedback("grasp", f"closing {start:.1f} -> {end:.2f} deg (floor) over {dur:.1f} s, contact expected at "
                               f"{self.expected_contact:.1f} deg, abort above {self.expected_contact + P.window_deg:.1f}")
        gh = self.motion.send_async("gripper", [[math.radians(end)]], [dur])
        res = gh.get_result_async()
        last = time.monotonic()
        ev = None
        deadline = last + dur + 3.0
        while ev is None and time.monotonic() < deadline:
            if self.motion.cancel_check():
                self.motion.stop_all()
                raise Canceled()
            for s in self.motion.samples_since("gripper", last):
                last = s.t
                ref, fb = math.degrees(s.ref["gripper"]), math.degrees(s.fb["gripper"])
                ev = det.update(s.t, ref, sg.simulated_measurement(ref, fb, p.sim_contact_deg), self.motion.gripper_load())
                if ev:
                    break
            if ev is None and res.done() and time.monotonic() - last > 0.3:
                break
            time.sleep(0.005)
        hold = end
        if ev is not None:
            if ev.angle_deg > self.expected_contact + P.window_deg:
                # freeze where the jaws are (no further squeeze); the caller opens again
                self.motion.run("gripper", [[math.radians(ev.angle_deg)]], times=[0.1], settle=False)
                return "early", NAN, ev, (f"contact at {ev.angle_deg:.1f} deg, expected {self.expected_contact:.1f} "
                                          f"+- {P.window_deg:.0f} (cube edge {p.cube_edge_mm:.0f} mm)")
            hold = sg.hold_target_deg(ev.angle_deg, P)              # >= the floor
            self.motion.run("gripper", [[math.radians(hold)]], times=[p.hold_ramp_s], settle=False)
            self.feedback("grasp", f"contact at {ev.angle_deg:.2f} deg (by {ev.by}): holding at {hold:.2f} deg")
        # judge the grip standing still: angle vs where empty jaws stop, and the current
        time.sleep(p.grip_settle_s)
        g = self.grip_measured_deg()
        cur = self.motion.gripper_current_ma(p.current_window_s)
        state, why = sg.grip_state(g, hold, P, cur)
        return state, hold, ev, why

    def back_off(self, plan, phase, why):
        """Open slowly and rise to the hover (after a miss at the bottom)."""
        self.feedback(phase, f"{why}: opening and backing up")
        self.gripper_to(self.p.open_deg, self.p.open_speed_deg_s)
        self.hook.restore()
        self.motion.run("arm", [r.q for r in plan.descent[::-1][1:]] + [plan.hover.q], self.p.lift_speed,
                        bias=self.arm_bias)

    def execute_pick(self, gh):
        res = PickCube.Result()
        res.contact_angle_deg = res.hold_angle_deg = res.held_angle_deg = NAN
        self.begin(gh, PickCube.Feedback)
        phase = "setup"
        try:
            self.check_preconditions()
            cube = cube_key(gh.request.cube_id)
            if self.holding is not None:
                # the record may be stale: someone took the cube out of the jaws by hand
                g = self.grip_measured_deg()
                state, why = sg.grip_state(g, self.holding["hold_deg"], self.sg,
                                           self.motion.gripper_current_ma(self.p.current_window_s))
                if state != sg.EMPTY:
                    raise Fail("setup", f"already holding {self.holding['cube']}: place it first ({why})")
                self.feedback("setup", f"{self.holding['cube']} is no longer between the jaws ({why})")
                self.holding = None
                self.hook.restore()
            p = self.p
            phase = "search"
            est = self.search(cube)
            plan = None
            for attempt in range(int(p.grasp_retries) + 1):
                phase = "approach"
                plan = self.plan_grasp(est, prefer_roll=None if plan is None else plan.roll)
                self.feedback("approach", plan.summary())
                if abs(math.degrees(self.motion.grip()) - p.open_deg) > 2:
                    q_now = self.motion.q_arm()
                    if self.model.lowest(q_now, math.radians(p.open_deg))[1] < 0.02:
                        raise Fail("approach", "too low to open the jaws here")
                    self.gripper_to(p.open_deg, p.open_speed_deg_s)
                self.transit(plan.hover.q, p.open_deg, "approach")
                phase = "refine"
                est, plan, shift = self.refine(cube, est, plan)
                res.refine_shift_mm = float(shift)
                res.cube_position = Point(x=float(est.centre[0]), y=float(est.centre[1]), z=float(est.centre[2]))
                res.cube_yaw_deg = float(math.degrees(est.yaw))
                res.wrist_roll_deg = float(math.degrees(plan.roll))
                res.jaw_tilt_deg = float(plan.grasp.tilt_deg)
                if gh.request.hover_only:
                    bias = self.motion.trim_arm(plan.hover.q, p.trim_tol_deg, int(p.trim_max_iter), p.trim_max_bias_deg)
                    self.feedback("refine", "hover trim " + ", ".join(
                        f"{n.split('_')[1]} {math.degrees(b):+.1f}" for n, b in zip(ARM, bias) if abs(b) > 1e-4))
                    res.success, res.message = True, f"hovering over {cube} (dry run): {plan.summary()}"
                    gh.succeed()
                    return res
                phase = "descend"
                # HOOK(torque-limit): 20 % during the grasp, confirmed by the readback; checked here
                # while the arm is still at the hover
                ok, why = self.hook.lower(is_mock=self.is_mock is True)
                self.feedback("descend", why)
                if not ok:
                    raise Fail("grasp", f"{why}: not grasping")
                self.descend(plan, p.open_deg)
                phase = "grasp"
                state, hold, ev, why = self.soft_close()
                if ev is not None:
                    res.contact_angle_deg = float(ev.angle_deg)
                if state == "early":
                    self.back_off(plan, "grasp", why)
                    if attempt < int(p.grasp_retries):
                        self.feedback("grasp", f"retry {attempt + 1}")
                        continue
                    raise Fail("grasp", why)
                res.hold_angle_deg = float(hold)
                phase = "verify"
                self.feedback("verify", f"{state}: {why}")
                if state != sg.HELD:
                    self.back_off(plan, "verify", "not held")
                    raise Fail("verify", f"not held: {why}")
                if ev is None:
                    res.contact_angle_deg = float(self.grip_measured_deg())
                self.holding = {"cube": cube, "hold_deg": hold, "contact_deg": res.contact_angle_deg,
                                "grasp_h": p.grasp_tip_height, "roll": plan.roll}
                phase = "lift"
                self.feedback("lift", f"to {plan.hover_h * 100:.1f} cm")
                r = self.motion.run("arm", [x.q for x in plan.descent[::-1][1:]] + [plan.hover.q], p.lift_speed,
                                    bias=self.arm_bias)
                if not r.ok:
                    raise Fail("lift", f"lift failed: {r.error}")
                time.sleep(0.3)
                g = self.grip_measured_deg()
                res.held_angle_deg = float(g)
                ok, why = sg.held(g, hold, self.sg, self.motion.gripper_current_ma(p.current_window_s))
                if not ok:
                    self.holding = None
                    raise Fail("lift", f"lost the cube while lifting: {why} (gripper left as it is)")
                res.success = True
                res.message = (f"holding {cube}: contact {res.contact_angle_deg:.2f} deg, hold command {hold:.2f}, "
                               f"measured {g:.2f} after the lift")
                self.feedback("lift", res.message)
                gh.succeed()
                return res
            raise Fail(phase, "out of retries")
        except Canceled:
            res.message, res.failed_phase = f"canceled during {phase}: motion stopped where it was", phase
            self.get_logger().warn(res.message)
            gh.canceled()
            return res
        except (Fail, MotionError) as e:
            res.failed_phase = getattr(e, "phase", phase)
            res.message = getattr(e, "message", str(e))
            self.get_logger().error(f"pick failed in {res.failed_phase}: {res.message}")
            gh.abort()
            return res
        finally:
            self.end()

    # --- place --------------------------------------------------------------------------------

    def execute_place(self, gh):
        res = PlaceCube.Result()
        self.begin(gh, PlaceCube.Feedback)
        phase = "setup"
        try:
            self.check_preconditions()
            if self.holding is None:
                # e.g. after a restart of this node with a cube still between the jaws: trust the grip
                # itself (same rule as the verify after a pick), never an uncertain one
                g = self.grip_measured_deg()
                state, why = sg.grip_state(g, self.floor_deg, self.sg,
                                           self.motion.gripper_current_ma(self.p.current_window_s))
                if state != sg.HELD:
                    raise Fail("setup", f"not holding a cube (PickCube first): {why}")
                self.holding = {"cube": "cube (held at start)", "hold_deg": self.floor_deg, "contact_deg": g,
                                "grasp_h": self.p.grasp_tip_height, "roll": self.motion.q_arm()[4]}
                self.feedback("setup", f"a cube is between the jaws: {why}")
            p, req, held = self.p, gh.request, self.holding
            grip = held["hold_deg"]
            yaws = None
            target = np.array([req.x, req.y], float)
            surface = 0.0
            on = req.on_cube_id.strip()
            if on:
                lower = self.tracker.get(on)
                if lower is None or lower.age_s > p.stack_max_age_s:
                    raise Fail("setup", f"{cube_key(on)} not seen in the last {p.stack_max_age_s:.0f} s")
                target = lower.centre[:2].copy()
                surface = float(lower.centre[2] + self.edge / 2 - self.model.table_z)
                yaws = self.planner.close_options(lower.yaw)
            elif not math.isnan(req.yaw_deg):
                yaws = self.planner.close_options(math.radians(req.yaw_deg))
            release_h = surface + held["grasp_h"] + p.release_gap
            hover_h = release_h + (p.hover_height - p.grasp_tip_height)

            def plan_at(xy, yaws_):
                heights = list(np.arange(hover_h, release_h + 0.03, -0.005))
                pl, reasons = self.planner.plan_vertical(xy, yaws_ if yaws_ is not None else [None], math.radians(grip),
                                                         heights, release_h, p.descend_step, 0.5,
                                                         prefer_roll=self.motion.q_arm()[4])
                if pl is None and p.allow_tilt:
                    pl, more = self.planner.plan_vertical(xy, yaws_ if yaws_ is not None else [None], math.radians(grip),
                                                          heights, release_h, p.descend_step, 0.5,
                                                          prefer_roll=self.motion.q_arm()[4], tilted=True)
                    reasons += more
                if pl is None:
                    raise Fail("carry", f"no place pose at ({xy[0] * 100:.1f}, {xy[1] * 100:.1f}) cm: "
                                        + " | ".join(reasons[:3]))
                return pl
            phase = "carry"
            plan = plan_at(target, yaws)
            self.feedback("carry", plan.summary())
            self.transit(plan.hover.q, grip, "carry", load_below=held["grasp_h"])
            if on:
                phase = "refine"
                new = self.wait_stable(on, time.monotonic() + p.obs_settle_s, p.refine_timeout_s + p.obs_settle_s)
                if new is not None:
                    shift = float(np.linalg.norm(new.centre[:2] - target) * 1000)
                    self.feedback("refine", f"{cube_key(on)} close-up shift {shift:.1f} mm")
                    if shift > p.refine_max_shift_mm:
                        raise Fail("refine", f"{cube_key(on)} moved {shift:.0f} mm: not stacking")
                    if shift > p.refine_min_shift_mm:
                        target = new.centre[:2].copy()
                        plan = plan_at(target, self.planner.close_options(new.yaw))
                        r = self.motion.run("arm", [plan.hover.q], p.approach_speed)
                        if not r.ok:
                            raise Fail("refine", r.error)
                else:
                    self.feedback("refine", "lower cube not seen from the hover: using the remembered pose")
            phase = "descend"
            i = self.descend(plan, grip, stop_on_contact_ok=True)
            phase = "release"
            self.feedback("release", "opening slowly")
            self.gripper_to(p.open_deg, p.open_speed_deg_s)
            self.hook.restore()
            q_rel = plan.descent[i].q
            tip = self.model.tip(q_rel)
            centre = tip + self.model.gap_offset(q_rel, math.radians(grip), 0.5)
            res.placed_position = Point(x=float(centre[0]), y=float(centre[1]),
                                        z=float(self.model.table_z + surface + self.edge / 2))
            self.holding = None
            phase = "retreat"
            self.feedback("retreat", "up to the hover")
            back = [r.q for r in plan.descent[:i][::-1]] + [plan.hover.q]
            r = self.motion.run("arm", back, p.lift_speed)
            if not r.ok:
                raise Fail("retreat", r.error)
            res.success = True
            res.message = (f"placed {held['cube']} at ({centre[0] * 100:.1f}, {centre[1] * 100:.1f}) cm"
                           + (f" on {cube_key(on)}" if on else ""))
            self.feedback("retreat", res.message)
            gh.succeed()
            return res
        except Canceled:
            res.message, res.failed_phase = f"canceled during {phase}: motion stopped, gripper unchanged", phase
            self.get_logger().warn(res.message)
            gh.canceled()
            return res
        except (Fail, MotionError) as e:
            res.failed_phase = getattr(e, "phase", phase)
            res.message = getattr(e, "message", str(e))
            self.get_logger().error(f"place failed in {res.failed_phase}: {res.message}")
            gh.abort()
            return res
        finally:
            self.end()

    # --- open gripper service ---------------------------------------------------------------------

    def on_open_gripper(self, _req, resp):
        if not self.busy.acquire(blocking=False):
            resp.success, resp.message = False, "busy with a pick/place goal"
            return resp
        try:
            q = self.motion.q_arm()
            if self.model.lowest(q, math.radians(self.p.open_deg))[1] < 0.0:
                resp.success, resp.message = False, "the open jaws would reach into the table here"
                return resp
            self.gripper_to(self.p.open_deg, self.p.open_speed_deg_s)
            self.hook.restore()
            self.holding = None
            resp.success, resp.message = True, f"gripper open at {self.p.open_deg:.0f} deg"
        except Exception as e:  # noqa: BLE001
            resp.success, resp.message = False, str(e)
        finally:
            self.busy.release()
        return resp


def sg_fold(a):
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def main():
    rclpy.init()
    node = PickPlaceNode()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.motion.stop_all()
        except Exception:  # noqa: BLE001  (context already shut down)
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
