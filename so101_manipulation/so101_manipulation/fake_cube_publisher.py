#!/usr/bin/env python3
"""FAKE cube detections for mock-hardware tests, shaped like the real cube detector's output:

  /cubes/detections  one Detection3D per frame the cube is "seen" in
  /cubes/tracked     (publish_tracked) a filtered estimate per cube like the detector's tracker:
                     header.stamp = last image the cube was seen in, score >= 0.5 when stable
                     (>= 5 obs, spread <= 2 mm / 3 deg), covariance[0] = spread_xy^2/2,
                     [35] = spread_yaw^2; the track restarts after 3 observations > 10 mm away

Like the real detector, frames are ignored while any arm joint moves faster than 0.1 rad/s and
for 0.3 s after.

Parameters: cube (k), x, y [m], yaw_deg, rate_hz, noise_mm, visible_cone_deg (the cube is only "seen"
while it lies within this angle of the wrist camera's optical axis, from the hand-eye file, to
exercise the search; <= 0: always), far_bias_mm [bx, by] added in far views (the far view is less
accurate, so the close-up refinement has something to correct); far = the fixed jaw tip higher
than far_height (0.10 m) above the table; publish_tracked (true).
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .grasp import load_hand_eye
from .kinematics import ARM, ArmModel, perception_config_dir


class FakeCubes(Node):
    def __init__(self):
        super().__init__("fake_cube_publisher")
        self.k = int(self.declare_parameter("cube", 2).value)
        self.x = float(self.declare_parameter("x", 0.22).value)
        self.y = float(self.declare_parameter("y", 0.03).value)
        self.yaw = math.radians(float(self.declare_parameter("yaw_deg", 20.0).value))
        rate = float(self.declare_parameter("rate_hz", 10.0).value)
        self.noise = float(self.declare_parameter("noise_mm", 0.5).value) / 1000
        self.vis = float(self.declare_parameter("visible_cone_deg", 0.0).value)
        self.bias = np.array(self.declare_parameter("far_bias_mm", [0.0, 0.0]).value, float) / 1000
        self.far_h = float(self.declare_parameter("far_height", 0.10).value)
        self.tracked = bool(self.declare_parameter("publish_tracked", True).value)
        self.model = ArmModel()
        self.T_gc = load_hand_eye(perception_config_dir() / "wrist_camera_hand_eye.yaml") if self.vis > 0 else None
        self.z = self.model.table_z + 0.015
        self.q, self.q_prev, self.moving_until = None, None, 0.0
        self.rng = np.random.default_rng(1)
        self.track = deque(maxlen=10)       # (centre, yaw)
        self.far_off = 0
        self.last_seen = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.pub = self.create_publisher(Detection3DArray, "/cubes/detections", 10)
        self.pub_tracked = self.create_publisher(Detection3DArray, "/cubes/tracked", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"FAKE cube_{self.k} at ({self.x * 100:.1f}, {self.y * 100:.1f}) cm, yaw "
                               f"{math.degrees(self.yaw):.0f} deg")

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if not all(n in d for n in ARM):
            return
        q, t = np.array([d[n] for n in ARM]), time.monotonic()
        if self.q_prev is not None and t > self.q_prev[1]:
            v = np.max(np.abs(q - self.q_prev[0])) / (t - self.q_prev[1])
            if v > 0.1:
                self.moving_until = t + 0.3
        self.q_prev = (q, t)
        self.q = list(q)

    def angle_from_axis(self):
        T = self.model.hand(self.q) @ self.T_gc
        v = np.array([self.x, self.y, self.z]) - T[:3, 3]
        return math.degrees(math.acos(float(np.clip(T[:3, 2] @ v / np.linalg.norm(v), -1, 1))))

    @staticmethod
    def fill(d, h, c, yaw):
        for pose in (h.pose.pose, d.bbox.center):
            pose.position.x, pose.position.y, pose.position.z = map(float, c)
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = 0.03

    def tick(self):
        now = self.get_clock().now().to_msg()
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = now, "base_link"
        still = time.monotonic() > self.moving_until
        if self.q is not None and still and (self.vis <= 0 or self.angle_from_axis() < self.vis):
            tip_z = self.model.tip(self.q)[2] - self.model.table_z
            c = np.array([self.x, self.y, self.z]) + np.r_[self.rng.normal(0, self.noise, 2), 0.0]
            if tip_z > self.far_h:
                c[:2] += self.bias
            yaw = (self.yaw + self.rng.normal(0, math.radians(0.5)) + math.pi / 4) % (math.pi / 2) - math.pi / 4
            d = Detection3D()
            d.header = arr.header
            d.id = f"cube_{self.k}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = d.id, 2 / 3
            self.fill(d, h, c, yaw)
            d.results.append(h)
            arr.detections.append(d)
            self.observe(c, yaw, now)
        self.pub.publish(arr)
        if self.tracked:
            self.publish_tracked()

    def observe(self, c, yaw, stamp):
        if self.track:
            med = np.median([o[0] for o in self.track], axis=0)
            self.far_off = self.far_off + 1 if np.linalg.norm(c[:2] - med[:2]) > 0.010 else 0
            if self.far_off >= 3:
                self.track.clear()
                self.far_off = 0
        self.track.append((c, yaw))
        self.last_seen = stamp

    def publish_tracked(self):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        if self.track:
            C = np.array([o[0] for o in self.track])
            med = np.median(C, axis=0)
            yaw = float(np.median([o[1] for o in self.track]))
            sxy = float(np.max(np.linalg.norm(C[:, :2] - med[:2], axis=1)))
            syaw = float(np.max(np.abs([o[1] - yaw for o in self.track])))
            stable = len(self.track) >= 5 and sxy <= 0.002 and syaw <= math.radians(3)
            d = Detection3D()
            d.header.stamp, d.header.frame_id = self.last_seen, "base_link"
            d.id = f"cube_{self.k}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = d.id
            h.hypothesis.score = 0.6 if stable else 0.3
            self.fill(d, h, med, yaw)
            h.pose.covariance[0] = h.pose.covariance[7] = sxy ** 2 / 2
            h.pose.covariance[35] = syaw ** 2
            d.results.append(h)
            arr.detections.append(d)
        self.pub_tracked.publish(arr)


def main():
    rclpy.init()
    n = FakeCubes()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
