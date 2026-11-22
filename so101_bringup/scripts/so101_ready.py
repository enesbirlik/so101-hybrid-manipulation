#!/usr/bin/env python3
"""Take the arm out of its folded rest so MoveIt can plan from there.

At rest the arm lies on its stops: shoulder_lift past the planning limit, elbow_flex on its
stop (the forearm touches the shoulder in the model), often the hand on the table. MoveIt
rejects such a start state. This sends arm_controller straight joint moves instead, in the
order of tools/so101_calibrate.py zero:

  1. elbow_flex opens to ELBOW_READY_DEG (lift into its planning range): the hand rises
     away from the table, nothing else moves
  2. with --zero: elbow_flex to 0, then the rest of the arm to 0

Moves at SPEED_RAD_S on the joint that moves furthest. Needs the control stack running
(so101_bringup so101_control.launch.py or the MoveIt demo).
"""

import argparse
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ELBOW_READY_DEG = 80.0
LIFT_MIN_DEG = -92.0      # inside the MoveIt limit (-97.4)
SPEED_RAD_S = 0.4


class Ready(Node):
    def __init__(self):
        super().__init__("so101_ready")
        self.state = {}
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def on_state(self, msg):
        self.state.update(zip(msg.name, msg.position))

    def current(self):
        while rclpy.ok() and not all(j in self.state for j in ARM):
            rclpy.spin_once(self, timeout_sec=0.1)
        return [self.state[j] for j in ARM]

    def move(self, target, label):
        start = self.current()
        largest = max(abs(t - s) for s, t in zip(start, target))
        if largest < math.radians(0.5):
            self.get_logger().info(f"{label}: already there")
            return True
        secs = max(2.0, largest / SPEED_RAD_S)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=target, time_from_start=Duration(sec=int(secs), nanosec=int(secs % 1 * 1e9)))]
        self.get_logger().info(f"{label}: {secs:.1f} s")
        self.client.wait_for_server()
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        if not sent.result().accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"{label}: {result.error_string or result.error_code}")
            return False
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zero", action="store_true", help="go on to the zero pose (elbow first)")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Ready()
    try:
        pan, lift, elbow, wflex, roll = node.current()
        ok = node.move([pan, max(lift, math.radians(LIFT_MIN_DEG)), min(elbow, math.radians(ELBOW_READY_DEG)), wflex, roll],
                       "unfold the elbow")
        if ok and args.zero:
            pan, lift, elbow, wflex, roll = node.current()
            ok = node.move([pan, lift, 0.0, wflex, roll], "elbow to zero") and \
                node.move([0.0] * 5, "arm to zero")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
