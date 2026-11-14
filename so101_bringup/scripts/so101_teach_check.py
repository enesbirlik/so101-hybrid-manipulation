#!/usr/bin/env python3
"""Check kinesthetic teaching: is a pose taught with the torque off the pose the arm holds and replays?

  sag     torque off, you hold the arm still; torque on; you let go. Prints how far each joint
          moved: right after torque on (the encoders should read the same, ~0) and after you let
          go (the servo's position error under gravity).
  record  torque off, you move the arm by hand; Enter saves the pose. Saves a YAML file.
  replay  torque on, the arm visits the recorded poses. Prints taught vs reached, per joint and
          at the tool point (gripper_frame_link, forward kinematics of /robot_description).

The encoder reading does not depend on the torque, so the joint calibration is the same in both
states. What can differ is mechanical: the servo stops short of its goal under load (P gain,
deadband), and play that the encoders do not see. This tool measures the first; the tool-point
numbers include neither the play nor the URDF error, so check those with a ruler (--pause).

Needs the control stack running (so101_control or demo.launch.py with use_mock_hardware:=false).
Results go to ~/so101_data/teach_check/.
"""

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from so101_lift import ARM, TOOL, Chain  # noqa: E402

TICKS_PER_RAD = 4096 / (2 * math.pi)
SAMPLE_S = 0.5            # averaging window for one pose reading
SETTLE_S = 1.5            # wait after a move before reading
SPEED_RAD_S = 0.3         # on the joint that moves furthest
MIN_MOVE_S = 2.0
OUT_DIR = Path.home() / "so101_data" / "teach_check"


class Teach(Node):
    def __init__(self):
        super().__init__("so101_teach_check")
        self.lock = threading.Lock()
        self.samples = None                   # list while sampling
        self.latest, self.urdf = {}, None
        self.create_subscription(JointState, "/joint_states", self.on_state, 50)
        self.create_subscription(String, "/robot_description", self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.torque_pub = self.create_publisher(Float64MultiArray, "/torque_controller/commands", 10)
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def on_state(self, msg):
        q = dict(zip(msg.name, msg.position))
        with self.lock:
            self.latest = q
            if self.samples is not None:
                self.samples.append([q[j] for j in ARM])

    def on_description(self, msg):
        self.urdf = msg.data

    def wait_inputs(self):
        while rclpy.ok() and (self.urdf is None or not all(j in self.latest for j in ARM)):
            time.sleep(0.1)

    def sample(self, secs=SAMPLE_S):
        """Mean and spread (std) of each arm joint over secs, rad."""
        with self.lock:
            self.samples = []
        time.sleep(secs)
        with self.lock:
            data, self.samples = np.array(self.samples), None
        return dict(zip(ARM, data.mean(axis=0))), dict(zip(ARM, data.std(axis=0)))

    def torque(self, on):
        self.torque_pub.publish(Float64MultiArray(data=[1.0 if on else 0.0]))
        if on:
            # The hardware holds the present pose until the controllers command it; restart them
            # so they start from where the arm is (as the dashboard's TORQUE ON does).
            time.sleep(0.3)
            self.restart_controllers()

    def restart_controllers(self):
        names = ["arm_controller", "gripper_controller"]
        for deactivate in (True, False):
            req = SwitchController.Request(strictness=SwitchController.Request.STRICT)
            if deactivate:
                req.deactivate_controllers = names
            else:
                req.activate_controllers = names
            req.timeout = Duration(sec=2)
            if not self.switch.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("controller_manager is not running")
            result = wait(self.switch.call_async(req))
            if not result.ok:
                raise RuntimeError("controller switch failed")

    def move(self, target):
        start = {j: self.latest[j] for j in ARM}
        largest = max(abs(target[j] - start[j]) for j in ARM)
        secs = max(MIN_MOVE_S, largest / SPEED_RAD_S)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=[target[j] for j in ARM], time_from_start=Duration(sec=int(secs), nanosec=int(secs % 1 * 1e9))))
        if not self.arm.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("arm_controller is not running")
        handle = wait(self.arm.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError("goal rejected")
        result = wait(handle.get_result_async()).result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.error_string or f"error {result.error_code}")


def wait(future):
    while not future.done():
        time.sleep(0.01)
    return future.result()


def ask(prompt):
    """input() that returns 'q' on Ctrl+D."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def report(chain, title, ref, got, spread=None):
    """Per-joint difference got - ref (deg, ticks) and the tool point difference (mm)."""
    print(f"  {title}")
    rows = {}
    for j in ARM:
        d = got[j] - ref[j]
        noise = f"  (noise {math.degrees(spread[j]):.2f})" if spread else ""
        print(f"    {j:<14}{math.degrees(d):+7.2f} deg {d * TICKS_PER_RAD:+6.0f} ticks{noise}")
        rows[j] = float(math.degrees(d))
    p0, _ = chain.pose(ref)
    p1, _ = chain.pose(got)
    dp = (p1 - p0) * 1000
    print(f"    tool point    dx {dp[0]:+6.1f}  dy {dp[1]:+6.1f}  dz {dp[2]:+6.1f}  |d| {np.linalg.norm(dp):5.1f} mm")
    rows["tool_mm"] = [float(v) for v in dp]
    return rows


def save(kind, data):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / f"{datetime.now():%Y%m%d-%H%M%S}-{kind}.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"saved {path}")
    return path


def as_floats(q):
    return {j: float(v) for j, v in q.items()}


def cmd_sag(node, chain, args):
    trials = []
    while True:
        node.torque(False)
        if ask("\nTORQUE OFF. Put the arm in a pose, hold it still (do not let it rest on anything), "
               "Enter [q quits] ") == "q":
            break
        held, spread = node.sample()
        node.torque(True)
        on, _ = node.sample()
        if ask("TORQUE ON. Let go of the arm, then Enter ") == "q":
            break
        time.sleep(0.5)
        free, _ = node.sample()
        print()
        trial = {"held": as_floats(held),
                 "torque_on": report(chain, "torque on, still held (encoders: should be ~0):", held, on, spread),
                 "let_go": report(chain, "after letting go (servo error under gravity):", held, free)}
        trials.append(trial)
    node.torque(True)
    if trials:
        save("sag", {"trials": trials})


def cmd_record(node, chain, args):
    poses = []
    node.torque(False)
    print("TORQUE OFF. Move the arm by hand; keep the poses in free air, above the table.")
    while True:
        key = ask(f"pose {len(poses) + 1}: hold still, Enter saves [q finishes] ")
        if key == "q":
            break
        q, spread = node.sample()
        p, _ = chain.pose(q)
        poses.append({"name": f"p{len(poses) + 1}", "joints": as_floats(q)})
        print(f"  saved, tool at ({p[0] * 100:.1f}, {p[1] * 100:.1f}, {p[2] * 100:.1f}) cm, "
              f"noise {max(math.degrees(s) for s in spread.values()):.2f} deg")
    ask("Support the arm, Enter to switch the torque on ")
    node.torque(True)
    if poses:
        path = save("poses", {"poses": poses})
        print(f"replay with: so101_teach_check.py replay {path}")


def cmd_replay(node, chain, args):
    poses = yaml.safe_load(Path(args.file).read_text())["poses"]
    for name, (lo, hi) in chain.limits.items():
        for pose in poses:
            if name in pose["joints"] and not lo <= pose["joints"][name] <= hi:
                print(f"{pose['name']}: {name} outside its limits, abort")
                return 1
    node.torque(True)
    results = []
    for rnd in range(args.rounds):
        for pose in poses:
            taught = pose["joints"]
            print(f"\n[{rnd + 1}/{args.rounds}] {pose['name']}: moving")
            node.move(taught)
            time.sleep(SETTLE_S)
            reached, spread = node.sample()
            row = {"round": rnd + 1, "pose": pose["name"], "reached": as_floats(reached),
                   "error": report(chain, "reached - taught:", taught, reached, spread)}
            if args.pause:
                answer = ask("  measure it (ruler), type a note or just Enter [q quits] ")
                if answer == "q":
                    results.append(row)
                    save("replay", {"file": str(args.file), "results": results})
                    return 0
                row["note"] = answer
            results.append(row)
    save("replay", {"file": str(args.file), "results": results})
    worst = max(np.linalg.norm(r["error"]["tool_mm"]) for r in results)
    print(f"\nworst tool point error from the joints alone: {worst:.1f} mm over {len(results)} visits")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("sag", help="taught pose vs held pose, one pose at a time")
    sub.add_parser("record", help="record poses by hand, torque off")
    rp = sub.add_parser("replay", help="visit recorded poses, torque on")
    rp.add_argument("file", help="poses YAML from record")
    rp.add_argument("--rounds", type=int, default=1, help="visit all poses this many times (repeatability)")
    rp.add_argument("--pause", action="store_true", help="wait at each pose, for a ruler measurement")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Teach()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.wait_inputs()
        chain = Chain(node.urdf, TOOL)
        return {"sag": cmd_sag, "record": cmd_record, "replay": cmd_replay}[args.cmd](node, chain, args) or 0
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"\n{e or 'interrupted'}; torque left as it is")
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
