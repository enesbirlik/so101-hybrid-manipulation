"""Gripper torque limit while grasping (HOOK(torque-limit)).

Interface (so101_hardware + gripper_torque_controller, added on a parallel branch):
  /gripper_torque_controller/commands  std_msgs/Float64MultiArray, [limit] in 0.1 % units
                                        ([200.0] = 20 %, [1000.0] = back to the configured limit)
  /dynamic_joint_states                 the gripper's `torque_limit` state interface (readback)

Real arm, 2026-11-22: a single publish right after creating the publisher was lost; publishing
repeatedly until the readback matches works. 20 % (200) holds a 30 mm paper cube.

So `apply(value)` publishes at 10 Hz until the gripper's torque_limit readback equals the value
(within 0.5), and gives up after `timeout_s`. The node lowers the limit at the hover (aborting
the pick if it does not apply) and restores it only after the gripper has opened again.

The soft grasp does not depend on this: the hard command floor bounds the force on its own.
"""

import time

from std_msgs.msg import Float64MultiArray


class GripperTorqueLimitHook:
    def __init__(self, node, motion, grasp_limit=200.0, normal_limit=1000.0, topic="/gripper_torque_controller/commands",
                 timeout_s=3.0, required=True):
        self.node, self.motion = node, motion
        self.grasp_limit, self.normal_limit = float(grasp_limit), float(normal_limit)
        self.timeout_s, self.required = float(timeout_s), bool(required)
        self.enabled = self.grasp_limit > 0
        self.pub = node.create_publisher(Float64MultiArray, topic, 10) if self.enabled else None
        self.topic = topic
        self.lowered = False

    def set_grasp_limit(self, value: float):
        """Torque limit for the next grasp (per grip profile: paper 20 %, wood 45 %)."""
        self.grasp_limit = float(value)
        self.enabled = self.grasp_limit > 0
        if self.enabled and self.pub is None:
            self.pub = self.node.create_publisher(Float64MultiArray, self.topic, 10)

    def available(self) -> bool:
        return self.pub is not None and self.pub.get_subscription_count() > 0 and \
            self.motion.gripper_torque_limit() is not None

    def apply(self, value: float) -> bool:
        """Publish [value] until the readback matches; True when it does within timeout_s."""
        end = time.monotonic() + self.timeout_s
        while time.monotonic() < end:
            self.pub.publish(Float64MultiArray(data=[float(value)]))
            time.sleep(0.1)
            got = self.motion.gripper_torque_limit()
            if got is not None and abs(got - value) <= 0.5:
                return True
        self.node.get_logger().error(f"gripper torque limit {value:.0f} not confirmed within {self.timeout_s:.0f} s "
                                     f"(readback {self.motion.gripper_torque_limit()})")
        return False

    def lower(self, is_mock=False) -> tuple:
        """(ok, message). ok False means: do not close the gripper."""
        if not self.enabled:
            return True, "torque limit off (gripper_torque_limit <= 0)"
        if not self.available():
            if is_mock or not self.required:
                return True, f"no {self.topic} / torque_limit state ({'mock' if is_mock else 'not required'}): skipped"
            return False, f"{self.topic} or the gripper torque_limit state is missing (torque_limit_required)"
        if not self.apply(self.grasp_limit):
            return False, f"gripper torque limit {self.grasp_limit:.0f} did not apply"
        self.lowered = True
        return True, f"gripper torque limit {self.grasp_limit:.0f} (x0.1 %) confirmed"

    def restore(self):
        """After the gripper has opened again. Never raises."""
        if not self.lowered:
            return
        try:
            if self.apply(self.normal_limit):
                self.lowered = False
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().error(f"torque limit restore failed: {e}")
