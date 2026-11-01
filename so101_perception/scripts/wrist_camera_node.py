#!/usr/bin/env python3
"""Wrist camera -> /wrist_camera/image_raw + /wrist_camera/camera_info.

The camera is mounted upside down. Like tools/so101_camera_calib.py, every frame is turned by
180 deg, so the image and the calibration (wrist_camera_fisheye.yaml, equidistant model) are
those of the upright image, in wrist_camera_optical_frame (x right, y down, z forward).
`scale` shrinks the published image (and its camera matrix) for RViz.
"""

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

DEFAULT_DEVICE = "/dev/v4l/by-id/usb-USB2.0_Camera_USB2.0_Camera_01.00.00-video-index0"


class WristCamera(Node):
    def __init__(self):
        super().__init__("wrist_camera")
        device = self.declare_parameter("device", DEFAULT_DEVICE).value
        calibration = self.declare_parameter("calibration_file", "").value
        self.frame_id = self.declare_parameter("frame_id", "wrist_camera_optical_frame").value
        self.scale = self.declare_parameter("scale", 0.5).value
        fps = self.declare_parameter("fps", 15.0).value

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {device}")

        self.info = self.camera_info(calibration) if calibration else None
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, "wrist_camera/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "wrist_camera/camera_info", qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self.grab)
        self.get_logger().info(f"{device} -> wrist_camera/image_raw at {fps:.0f} fps, scale {self.scale}")

    def camera_info(self, path):
        with open(path) as f:
            c = yaml.safe_load(f)
        s = self.scale
        info = CameraInfo()
        info.width, info.height = round(c["image_width"] * s), round(c["image_height"] * s)
        info.distortion_model = c["distortion_model"]
        info.d = list(c["distortion_coefficients"]["data"])
        k = list(c["camera_matrix"]["data"])
        k[0], k[2], k[4], k[5] = k[0] * s, k[2] * s, k[4] * s, k[5] * s
        info.k = k
        info.r = list(c["rectification_matrix"]["data"])
        p = list(c["projection_matrix"]["data"])
        p[0], p[2], p[5], p[6] = p[0] * s, p[2] * s, p[5] * s, p[6] * s
        info.p = p
        return info

    def grab(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("no frame", throttle_duration_sec=2.0)
            return
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub_image.publish(msg)
        if self.info is not None:
            self.info.header = msg.header
            self.pub_info.publish(self.info)


def main():
    rclpy.init()
    node = WristCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
