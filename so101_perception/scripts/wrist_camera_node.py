#!/usr/bin/env python3
"""Wrist camera -> /wrist_camera/image_raw + /wrist_camera/camera_info + /wrist_camera/fov.

The camera is mounted upside down. Like tools/so101_camera_calib.py, every frame is turned by
180 deg, so the image and the calibration (wrist_camera_fisheye.yaml, equidistant model) are
those of the upright image, in wrist_camera_optical_frame (x right, y down, z forward).

Light on the CPU: the MJPG frames are taken undecoded and decoded straight at the published
size (`scale` 0.5 or 0.25 decodes at 1/2 or 1/4 resolution), and only while someone subscribes
to the image. The field of view is drawn as a marker from the fisheye calibration.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from visualization_msgs.msg import Marker

DEFAULT_DEVICE = "/dev/v4l/by-id/usb-USB2.0_Camera_USB2.0_Camera_01.00.00-video-index0"
WIDTH, HEIGHT = 1280, 720
# decode straight at a reduced size where libjpeg can
REDUCED = {1.0: cv2.IMREAD_COLOR, 0.5: cv2.IMREAD_REDUCED_COLOR_2, 0.25: cv2.IMREAD_REDUCED_COLOR_4}
FOV_DEPTH = 0.15      # m, length of the drawn field-of-view rays
FOV_EDGE_POINTS = 12  # samples per image edge: the fisheye footprint is curved


class WristCamera(Node):
    def __init__(self):
        super().__init__("wrist_camera")
        device = self.declare_parameter("device", DEFAULT_DEVICE).value
        calibration = self.declare_parameter("calibration_file", "").value
        self.frame_id = self.declare_parameter("frame_id", "wrist_camera_optical_frame").value
        self.scale = float(self.declare_parameter("scale", 0.5).value)
        fps = self.declare_parameter("fps", 15.0).value

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        # no CAP_PROP_BUFFERSIZE 1: with one buffer the driver drops every other frame (25 -> 13 fps);
        # the capture thread takes every frame anyway, so none goes stale
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # hand over the JPEG bytes undecoded
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {device}")

        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, "wrist_camera/image_raw", qos_profile_sensor_data)
        # image_transport naming, so RViz can pick it with the "compressed" transport hint
        self.pub_jpeg = self.create_publisher(CompressedImage, "wrist_camera/image_raw/compressed", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "wrist_camera/camera_info", qos_profile_sensor_data)
        self.pub_fov = self.create_publisher(Marker, "wrist_camera/fov", 1)
        self.info, self.fov = None, None
        if calibration:
            K, D, self.info = self.load_calibration(calibration)
            self.fov = self.fov_marker(K, D)
            self.create_timer(1.0, self.publish_fov)
        self.period = 1.0 / fps
        threading.Thread(target=self.capture_loop, daemon=True).start()
        self.get_logger().info(f"{device} -> wrist_camera/image_raw at {fps:.0f} fps, scale {self.scale}")

    def load_calibration(self, path):
        with open(path) as f:
            c = yaml.safe_load(f)
        K = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
        D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
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
        return K, D, info

    def fov_marker(self, K, D):
        """Rays through the image border (fisheye model), FOV_DEPTH long, and the outline they span."""
        t = np.linspace(0, 1, FOV_EDGE_POINTS, endpoint=False)
        w, h = WIDTH - 1, HEIGHT - 1
        border = np.concatenate([
            np.c_[t * w, np.zeros_like(t)], np.c_[np.full_like(t, w), t * h],
            np.c_[(1 - t) * w, np.full_like(t, h)], np.c_[np.zeros_like(t), (1 - t) * h]])
        xy = cv2.fisheye.undistortPoints(border.reshape(-1, 1, 2).astype(np.float64), K, D).reshape(-1, 2)
        rays = np.c_[xy, np.ones(len(xy))]
        rays = rays / np.linalg.norm(rays, axis=1, keepdims=True) * FOV_DEPTH
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns, m.id, m.type, m.action = "wrist_camera_fov", 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.001
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 0.8
        m.pose.orientation.w = 1.0
        apex = Point()
        corners = [0, FOV_EDGE_POINTS, 2 * FOV_EDGE_POINTS, 3 * FOV_EDGE_POINTS]
        for i in range(len(rays)):
            a, b = rays[i], rays[(i + 1) % len(rays)]
            m.points += [Point(x=a[0], y=a[1], z=a[2]), Point(x=b[0], y=b[1], z=b[2])]
            if i in corners:
                m.points += [apex, Point(x=a[0], y=a[1], z=a[2])]
        return m

    def publish_fov(self):
        self.fov.header.stamp = self.get_clock().now().to_msg()
        self.pub_fov.publish(self.fov)

    def capture_loop(self):
        """Takes every frame at the camera's own rate (so none goes stale in its buffer) and
        decodes and publishes one per period, only while someone listens."""
        due = 0.0
        while rclpy.ok():
            if not self.cap.grab():
                self.get_logger().warn("no frame", throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            now = time.monotonic()
            listeners = self.pub_image.get_subscription_count() + self.pub_jpeg.get_subscription_count()
            if now < due or listeners == 0:
                continue
            # on a schedule, not 'period since the last one': with 25 fps frames that would round
            # every gap up to two frames (12.5 Hz instead of 15)
            due = max(due + self.period, now - self.period)
            self.publish_frame()

    def publish_frame(self):
        ok, buf = self.cap.retrieve()
        if not ok:
            return
        flag = REDUCED.get(self.scale)
        if buf.ndim == 3:   # the driver decoded it after all
            frame = buf
        else:
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR if flag is None else flag)
            if frame is None:
                return
        if frame.shape[1] != round(WIDTH * self.scale):
            frame = cv2.resize(frame, (round(WIDTH * self.scale), round(HEIGHT * self.scale)), interpolation=cv2.INTER_AREA)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        if self.pub_image.get_subscription_count():
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = header
            self.pub_image.publish(msg)
        if self.pub_jpeg.get_subscription_count():
            # ~35 kB at 640x360 instead of 690 kB raw, which best-effort DDS drops under load
            msg = CompressedImage(header=header, format="bgr8; jpeg compressed bgr8",
                                  data=cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
            self.pub_jpeg.publish(msg)
        if self.info is not None:
            self.info.header = header
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
