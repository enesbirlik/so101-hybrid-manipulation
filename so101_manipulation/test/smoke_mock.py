#!/usr/bin/env python3
"""Integration smoke test on MOCK hardware (not run by colcon test: it needs a running stack).

  export ROS_DOMAIN_ID=42
  ros2 launch so101_moveit_config demo.launch.py rviz:=false &      # mock hardware by default
  python3 test/smoke_mock.py [--only NAME]

It starts its own FAKE cube publisher and pick_place node per scenario (and stops them again):

  miss       no simulated contact: PickCube runs search .. verify and ends cleanly "not held"
             (with cube_source:=detections, the /cubes/detections fallback; the others use
             /cubes/tracked through cube_source:=auto)
  pick_place simulated cube blocking the jaws at 12.5 deg (where the real arm held a paper cube): PickCube succeeds, PlaceCube puts it down
  search     the cube is only "visible" from a pan away from the start: the view sweep finds it,
             and a far-view bias of 6 mm is corrected by the close-up refinement
  reach      cube 40 cm out: rejected in approach with a reach message
  cancel     cancel during the approach: the goal ends canceled, the arm stops
  overview   ~/overview (cube seen through the fisheye model, fov_check): JSON answer, ~/state, then a
             pick that skips the search (the cube is fresh and stable)
  memory     a pick that had to sweep, the overview (cube out of view), a second pick: its search
             starts with the view aimed at where the cube was last seen, and is faster

Refuses to run on ROS_DOMAIN_ID 0 / unset (the real arm lives there).
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
    sys.exit("refusing: set ROS_DOMAIN_ID (e.g. 42) to the mock stack's domain; the real arm runs on 0")

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from control_msgs.msg import JointTrajectoryControllerState  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402

from so101_manipulation_interfaces.action import PickCube, PlaceCube  # noqa: E402

PHASES_PICK = ["search", "approach", "refine", "descend", "grasp", "verify", "lift"]


def start(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL if not os.environ.get("SMOKE_VERBOSE") else None,
                            stderr=subprocess.STDOUT if not os.environ.get("SMOKE_VERBOSE") else None)


def stop(procs):
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


class Driver(Node):
    def __init__(self):
        super().__init__("smoke_mock_driver")
        self.pick = ActionClient(self, PickCube, "/pick_cube")
        self.place = ActionClient(self, PlaceCube, "/place_cube")
        self.js = {}
        self.desc = None
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.min_grip_cmd = math.inf      # lowest gripper reference seen on the controller state
        self.create_subscription(JointTrajectoryControllerState, "/gripper_controller/controller_state", self.on_gs, 50)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "desc", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.overview = self.create_client(Trigger, "/cube_pick_place/overview")
        self.state = None
        self.create_subscription(String, "/cube_pick_place/state", lambda m: setattr(self, "state", json.loads(m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def call_overview(self, timeout=60):
        assert self.overview.wait_for_service(timeout_sec=20), "no overview service"
        fut = self.overview.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        assert fut.done(), "overview: no answer in time"
        return fut.result()

    def settle_state(self, s=0.5):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.state

    def on_gs(self, m):
        # mock hardware starts with the gripper at 0 deg (below the floor); count the commands
        # once the node has moved the gripper above the floor
        if m.reference.positions:
            r = m.reference.positions[0]
            if self.min_grip_cmd == math.inf and r < math.radians(20.0):   # not yet opened once
                return
            self.min_grip_cmd = min(self.min_grip_cmd, r)

    def call(self, client, goal, cancel_after_phase=None, timeout=240):
        phases = []
        self.details = []

        def fb(m):
            f = m.feedback
            if not phases or phases[-1] != f.phase:
                phases.append(f.phase)
            self.details.append(f.detail)
            print(f"    [{f.phase}] {f.detail}")
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError("action server not up")
        fut = client.send_goal_async(goal, feedback_callback=fb)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        gh = fut.result()
        assert gh.accepted
        res = gh.get_result_async()
        t0 = time.time()
        canceled = False
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cancel_after_phase and not canceled and cancel_after_phase in phases:
                time.sleep(0.5)
                c = gh.cancel_goal_async()
                rclpy.spin_until_future_complete(self, c, timeout_sec=5)
                canceled = True
        assert res.done(), "no result in time"
        return res.result().status, res.result().result, phases


def spawn(prefix, fake, node_params):
    lib = os.path.join(prefix, "lib", "so101_manipulation")
    procs = [start([os.path.join(lib, "fake_cube_publisher"), "--ros-args"] + sum((["-p", f"{k}:={v}"] for k, v in fake.items()), []))]
    cfg = os.path.join(prefix, "share", "so101_manipulation", "config", "pick_place.yaml")
    procs.append(start([os.path.join(lib, "pick_place"), "--ros-args", "-r", "__node:=cube_pick_place", "--params-file", cfg]
                       + sum((["-p", f"{k}:={v}"] for k, v in node_params.items()), [])))
    return procs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--prefix", default=os.environ.get("SO101_MANIP_PREFIX", ""),
                    help="install prefix of so101_manipulation (default: from AMENT_PREFIX_PATH)")
    a = ap.parse_args()
    prefix = a.prefix
    if not prefix:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix("so101_manipulation")
    rclpy.init()
    d = Driver()
    end = time.time() + 15
    while (d.desc is None or len(d.js) < 6) and time.time() < end:
        rclpy.spin_once(d, timeout_sec=0.1)
    if d.desc is None or "mock_components/GenericSystem" not in d.desc:
        sys.exit("refusing: /robot_description is not mock hardware (or no stack on this domain)")
    from rclpy.action.client import GoalStatus
    S = GoalStatus
    results = {}

    def scenario(name, fake, params, fn):
        if a.only and a.only != name:
            return
        print(f"=== {name}")
        procs = spawn(prefix, fake, params)
        d.min_grip_cmd = math.inf
        try:
            time.sleep(4.0)
            ok, why = fn()
        except AssertionError as e:
            ok, why = False, f"assertion: {e}"
        except Exception as e:  # noqa: BLE001
            ok, why = False, f"{type(e).__name__}: {e}"
        finally:
            stop(procs)
            time.sleep(1.0)
        results[name] = (ok, why)
        print(f"=== {name}: {'PASS' if ok else 'FAIL'} {why}")

    def miss():
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2"))
        assert st == S.STATUS_ABORTED, f"status {st}"
        assert r.failed_phase == "verify" and "not held" in r.message, r.message
        for p in ["search", "approach", "refine", "descend", "grasp"]:
            assert p in ph, f"phase {p} missing: {ph}"
        floor = (30.0 - 1.5 - 13.0) / 1.35
        assert d.min_grip_cmd >= math.radians(floor) - 1e-6, f"gripper command {math.degrees(d.min_grip_cmd):.2f} below the floor"
        return True, f"phases {ph}; lowest gripper command {math.degrees(d.min_grip_cmd):.2f} deg; '{r.message}'"

    def pick_place():
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="cube_2"))
        assert st == S.STATUS_SUCCEEDED, f"pick status {st}: {r.message}"
        assert [p for p in PHASES_PICK if p in ph] == PHASES_PICK, ph
        assert abs(r.contact_angle_deg - 12.5) < 0.3, r.contact_angle_deg
        floor = (30.0 - 1.5 - 13.0) / 1.35
        assert abs(r.hold_angle_deg - floor) < 0.01, r.hold_angle_deg
        assert d.min_grip_cmd >= math.radians(floor) - 1e-6, f"gripper command {math.degrees(d.min_grip_cmd):.2f} deg below the floor"
        assert abs(r.wrist_roll_deg) <= 90
        assert "timing [s]: " in r.message and "total" in r.message, r.message
        s = d.settle_state()
        assert s and s["state"] == "idle" and s["holding"] == "cube_2", s
        assert s["last"]["action"] == "pick" and s["last"]["success"] and "lift" in s["last"]["timing_s"], s
        st2, r2, ph2 = d.call(d.place, PlaceCube.Goal(x=0.25, y=-0.05, on_cube_id="", yaw_deg=float("nan")))
        assert st2 == S.STATUS_SUCCEEDED, f"place status {st2}: {r2.message}"
        for p in ["carry", "descend", "release", "retreat"]:
            assert p in ph2, ph2
        assert math.hypot(r2.placed_position.x - 0.25, r2.placed_position.y + 0.05) < 0.003, r2.placed_position
        g = math.degrees(d.js.get("gripper", 0))
        assert abs(g - 30) < 1.5, f"gripper {g:.1f} after release"
        s = d.settle_state()
        assert s["holding"] is None and s["last"]["action"] == "place", s
        return True, (f"pick: contact {r.contact_angle_deg:.1f}, hold {r.hold_angle_deg:.1f}, held {r.held_angle_deg:.1f}, "
                      f"roll {r.wrist_roll_deg:.0f}, tilt {r.jaw_tilt_deg:.1f}; {r.message.split('|')[-1].strip()}; "
                      f"place: {r2.message}")

    def overview():
        t0 = time.time()
        resp = d.call_overview()
        dt = time.time() - t0
        out = json.loads(resp.message)
        assert resp.success, out
        cubes = {c["id"]: c for c in out["cubes"]}
        assert "cube_2" in cubes, out
        c = cubes["cube_2"]
        assert math.hypot(c["x_cm"] - 27.0, c["y_cm"] + 10.0) < 0.5 and c["stable"], c
        q = [math.degrees(d.js[n]) for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]
        assert abs(q[4] - (-72.4)) < 1.0, q
        s = d.settle_state()
        assert s["state"] == "idle" and s["holding"] is None, s
        # the cube is fresh and stable from the overview: the pick does not search
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2", hover_only=True))
        assert st == S.STATUS_SUCCEEDED, r.message
        search_s = float(r.message.split("search ")[1].split(",")[0])
        assert search_s < 0.5, r.message
        return True, f"overview {dt:.1f} s: {out}; pick: {r.message.split('|')[-1].strip()}"

    def memory():
        # the cube is only "seen" within 10 deg of the optical axis: the first pick sweeps for it;
        # then the arm goes to the overview (cube out of view) and the next search starts with the
        # view aimed at where the cube was last seen
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2", hover_only=True))
        assert st == S.STATUS_SUCCEEDED, r.message
        first = float(r.message.split("search ")[1].split(",")[0])
        resp = d.call_overview()
        out = json.loads(resp.message)
        assert not any(c["id"] == "cube_2" for c in out["cubes"]), out
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2", hover_only=True))
        assert st == S.STATUS_SUCCEEDED, r.message
        second = float(r.message.split("search ")[1].split(",")[0])
        found = [x for x in d.details if "found from view" in x]
        assert found and "aimed at" in found[0], d.details
        assert second < first, (first, second)
        return True, f"search {first:.1f} s without memory, {second:.1f} s with it ({found[0]})"

    def search():
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2", hover_only=True))
        assert st == S.STATUS_SUCCEEDED, f"status {st}: {r.message}"
        assert r.refine_shift_mm > 4.0, f"refine shift {r.refine_shift_mm:.1f} mm (bias 6 mm)"
        assert math.hypot(r.cube_position.x - 0.20, r.cube_position.y + 0.12) < 0.002
        return True, f"found after sweep, close-up corrected {r.refine_shift_mm:.1f} mm; {r.message}"

    def reach():
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2"))
        assert st == S.STATUS_ABORTED and r.failed_phase == "approach" and "reach" in r.message, r.message
        return True, r.message[:120]

    def cancel():
        st, r, ph = d.call(d.pick, PickCube.Goal(cube_id="2"), cancel_after_phase="approach")
        assert st == S.STATUS_CANCELED, f"status {st}: {r.message}"
        # a cancel in the middle of a move may coast a little (the controller's stop), then holds
        q0 = dict(d.js)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            rclpy.spin_once(d, timeout_sec=0.05)
        q1 = dict(d.js)
        coast = max(abs(q1[k] - q0[k]) for k in q0)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(d, timeout_sec=0.05)
        moved = max(abs(d.js[k] - q1[k]) for k in q1)
        who = max(q1, key=lambda k: abs(d.js[k] - q1[k]))
        assert coast < math.radians(3.0), f"arm went on after cancel ({math.degrees(coast):.1f} deg)"
        assert moved < 0.01, f"arm still moving 0.5 s after cancel ({who} {math.degrees(moved):.1f} deg)"
        return True, f"{r.message} (coast {math.degrees(coast):.1f} deg)"

    base = {"x": 0.22, "y": 0.03, "yaw_deg": 20.0}
    # the fallback path: per-frame /cubes/detections only, the node's own median window
    scenario("miss", {**base, "publish_tracked": "false"}, {"cube_source": "detections"}, miss)
    scenario("pick_place", base, {"sim_contact_deg": 12.5}, pick_place)
    scenario("search", {"x": 0.20, "y": -0.12, "yaw_deg": -30.0, "visible_cone_deg": 10.0, "far_bias_mm": "[6.0, 0.0]"},
             {}, search)
    scenario("reach", {"x": 0.40, "y": 0.0, "yaw_deg": 0.0}, {}, reach)
    scenario("cancel", base, {}, cancel)
    scenario("overview", {"x": 0.27, "y": -0.10, "yaw_deg": 10.0, "fov_check": "true"}, {}, overview)
    scenario("memory", {"x": 0.20, "y": 0.08, "yaw_deg": -30.0, "visible_cone_deg": 10.0}, {}, memory)
    d.destroy_node()
    rclpy.try_shutdown()
    print("\nsummary:")
    for k, (ok, why) in results.items():
        print(f"  {k:<11}{'PASS' if ok else 'FAIL'}  {why}")
    return 0 if all(ok for ok, _ in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
