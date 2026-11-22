#!/usr/bin/env python3
"""Lift the hand straight up, without MoveIt: for a hand resting on (or pressed against) the table.

With the hand on the table the model puts it a few millimetres to centimetres inside the table
(the joint play the encoders do not see, devlog day 4), and MoveIt refuses to plan from a start
state in collision. This moves the tool point (gripper_frame_link) straight up by --dz, keeping
its x, y and the gripper orientation, so the jaws do not sweep the table. Only shoulder_lift,
elbow_flex and wrist_flex move. The joint path comes from forward kinematics of
/robot_description and is sent to arm_controller as one trajectory.

Moves at SPEED_RAD_S on the joint that moves furthest. Needs the control stack running.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MOVING = ["shoulder_lift", "elbow_flex", "wrist_flex"]
TOOL = "gripper_frame_link"
STEPS = 20                # waypoints along the lift
SPEED_RAD_S = 0.4
ROT_WEIGHT = 0.1          # m per rad: how much the orientation counts against the position
TOLERANCE_M = 0.002       # largest tool position error accepted along the path


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_matrix(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


class Chain:
    """Joints from the URDF root to one link, for forward kinematics."""

    def __init__(self, urdf, tip):
        root = ET.fromstring(urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        self.joints, self.limits = [], {}
        link = tip
        while link in by_child:
            j = by_child[link]
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            a = j.find("axis")
            axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
            self.joints.insert(0, (j.get("name"), j.get("type"), np.array(xyz), rpy_matrix(*rpy), axis))
            lim = j.find("limit")
            if lim is not None and j.get("type") == "revolute":
                self.limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
            link = j.find("parent").get("link")

    def pose(self, q):
        R, p = np.eye(3), np.zeros(3)
        for name, kind, xyz, rot, axis in self.joints:
            p = p + R @ xyz
            R = R @ rot
            if kind in ("revolute", "continuous"):
                R = R @ axis_matrix(axis, q.get(name, 0.0))
        return p, R


def rotation_error(R_target, R):
    """Axis-angle vector that turns R into R_target (world frame)."""
    E = R_target @ R.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1) / 2)))
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return v * angle / (2 * math.sin(angle))


def solve(chain, q, p_target, R_target, iterations=30):
    """Damped least squares on the MOVING joints towards a tool position and orientation."""
    q = dict(q)
    for _ in range(iterations):
        p, R = chain.pose(q)
        err = np.concatenate([p_target - p, ROT_WEIGHT * rotation_error(R_target, R)])
        if np.linalg.norm(err[:3]) < 1e-4 and np.linalg.norm(err[3:]) < 1e-4:
            break
        J = np.zeros((6, len(MOVING)))
        for k, name in enumerate(MOVING):
            dq = dict(q)
            dq[name] += 1e-5
            p2, R2 = chain.pose(dq)
            J[:3, k] = (p2 - p) / 1e-5
            J[3:, k] = ROT_WEIGHT * rotation_error(R2, R) / 1e-5
        step = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), err)
        for k, name in enumerate(MOVING):
            q[name] += step[k]
    p, _ = chain.pose(q)
    return q, np.linalg.norm(p_target - p)


class Lift(Node):
    def __init__(self):
        super().__init__("so101_lift")
        self.state, self.urdf = {}, None
        self.create_subscription(JointState, "/joint_states", lambda m: self.state.update(zip(m.name, m.position)), 10)
        self.create_subscription(String, "/robot_description", self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def on_description(self, msg):
        self.urdf = msg.data

    def wait_inputs(self):
        while rclpy.ok() and (self.urdf is None or not all(j in self.state for j in ARM)):
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self, path, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for k, q in enumerate(path, start=1):
            t = secs * k / len(path)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[q[j] for j in ARM], time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
        self.client.wait_for_server()
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        if not sent.result().accepted:
            self.get_logger().error("goal rejected")
            return False
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(result.error_string or str(result.error_code))
            return False
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dz", type=float, default=4.0, help="how far up, cm (default 4)")
    parser.add_argument("--dry-run", action="store_true", help="print the path, do not move")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Lift()
    try:
        node.wait_inputs()
        chain = Chain(node.urdf, TOOL)
        start = {j: node.state[j] for j in ARM}
        p0, R0 = chain.pose(start)
        path, q = [], start
        for k in range(1, STEPS + 1):
            q, miss = solve(chain, q, p0 + np.array([0.0, 0.0, args.dz / 100 * k / STEPS]), R0)
            if miss > TOLERANCE_M:
                node.get_logger().error(f"cannot lift {args.dz:.1f} cm from here (stuck at {args.dz * (k - 1) / STEPS:.1f} cm)")
                return 1
            for name in MOVING:
                lo, hi = chain.limits.get(name, (-math.inf, math.inf))
                if not lo <= q[name] <= hi:
                    node.get_logger().error(f"{name} would leave its limits at {args.dz * k / STEPS:.1f} cm")
                    return 1
            path.append(q)
        largest = max(abs(path[-1][j] - start[j]) for j in MOVING)
        secs = max(1.5, largest / SPEED_RAD_S)
        p1, _ = chain.pose(path[-1])
        change = ", ".join(f"{j} {math.degrees(path[-1][j] - start[j]):+.1f} deg" for j in MOVING)
        node.get_logger().info(f"tool z {p0[2] * 100:.1f} -> {p1[2] * 100:.1f} cm in {secs:.1f} s ({change})")
        if args.dry_run:
            return 0
        return 0 if node.send(path, secs) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
