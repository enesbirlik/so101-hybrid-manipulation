#!/usr/bin/env python3
"""ArUco cubes in the wrist camera -> their poses in base_link.

Subscribes  wrist_camera/image_raw/compressed (any size; K is scaled to it)
Publishes   cubes/detections          vision_msgs/Detection3DArray, cubes seen in this frame
            cubes/markers             visualization_msgs/MarkerArray, seen (solid) and remembered (faint)
            wrist_camera/cubes/compressed  the image with markers and cube outlines, only while subscribed
            TF base_link -> cube_<k>  for every cube seen in the last `memory_s` seconds

Geometry in cube_detection.py. A cube resting on the table has its centre at table_z + 15 mm;
that height is used to fix the distance along the camera ray (`snap_to_table`).
"""

import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

import cube_detection as cd

# colours of the printed sheet (K0..K5), RGB 0..1
CUBE_COLOURS = [(0.96, 0.55, 0.55), (0.45, 0.70, 0.95), (0.50, 0.85, 0.50),
                (0.98, 0.75, 0.40), (0.70, 0.55, 0.95), (0.95, 0.50, 0.80)]
_e = cd.CUBE_EDGE / 2
CUBE_CORNERS = np.array([[x, y, z] for x in (-_e, _e) for y in (-_e, _e) for z in (-_e, _e)])
CUBE_EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_R(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        perception = get_package_share_directory("so101_perception")
        calib = self.declare_parameter(
            "calibration_file", os.path.join(perception, "config", "wrist_camera_fisheye.yaml")).value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        image_topic = self.declare_parameter("image_topic", "wrist_camera/image_raw/compressed").value
        self.memory_s = float(self.declare_parameter("memory_s", 30.0).value)
        self.snap = bool(self.declare_parameter("snap_to_table", True).value)
        self.max_reproj = float(self.declare_parameter("max_reproj_px", 3.0).value)
        table_z = self.declare_parameter("table_z", float("nan")).value
        if np.isnan(table_z):
            table_z = self.table_z_from_description()
        self.table_centre_z = table_z + cd.CUBE_EDGE / 2

        with open(calib) as f:
            c = yaml.safe_load(f)
        self.K_full = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
        self.D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
        self.calib_width = c["image_width"]
        self.K, self.K_width = None, None

        self.detector = cd.MarkerDetector()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_det = self.create_publisher(Detection3DArray, "cubes/detections", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "cubes/markers", 10)
        self.pub_image = self.create_publisher(CompressedImage, "wrist_camera/cubes/compressed", qos_profile_sensor_data)
        self.create_subscription(CompressedImage, image_topic, self.on_image, qos_profile_sensor_data)
        self.known = {}          # cube -> (CubeEstimate, wall time seen)
        self.create_timer(0.2, self.publish_memory)
        self.get_logger().info(
            f"{image_topic} -> cubes/detections in {self.target_frame}, table centre z {self.table_centre_z * 1000:.1f} mm"
            f"{'' if self.snap else ' (not used)'}")

    def table_z_from_description(self):
        try:
            path = os.path.join(get_package_share_directory("so101_description"), "config", "so101_kinematics.yaml")
            with open(path) as f:
                return float(yaml.safe_load(f)["table_z"])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no table_z in so101_kinematics.yaml ({e}); using 0")
            return 0.0

    def camera_pose(self, frame, stamp):
        """target <- camera at the image time, or the latest one if the arm's TF is not there yet."""
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame, t, timeout=Duration(seconds=0.05))
                tr = tf.transform.translation
                return quat_to_R(tf.transform.rotation), np.array([tr.x, tr.y, tr.z])
            except Exception:  # noqa: BLE001
                continue
        self.get_logger().warn(f"no TF {self.target_frame} <- {frame}", throttle_duration_sec=2.0)
        return None

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if self.K_width != img.shape[1]:
            self.K, self.K_width = cd.scaled_camera(self.K_full, img.shape[1], self.calib_width), img.shape[1]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        poses = cd.marker_poses(detections, self.K, self.D)
        pose = self.camera_pose(msg.header.frame_id, msg.header.stamp) if poses else None
        cubes = []
        if pose is not None:
            R_wc, t_wc = pose
            cubes = cd.cube_estimates(poses, R_wc, t_wc, max_reproj_px=self.max_reproj,
                                      table_centre_z=self.table_centre_z if self.snap else None)
            now = time.monotonic()
            for e in cubes:
                self.known[e.cube] = (e, now)
        self.publish_detections(msg.header.stamp, cubes)
        if self.pub_image.get_subscription_count():
            self.publish_overlay(msg, img, detections, poses, cubes, pose)

    def publish_detections(self, stamp, cubes):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = stamp, self.target_frame
        for e in cubes:
            d = Detection3D()
            d.header = arr.header
            d.id = f"cube_{e.cube}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = f"cube_{e.cube}"
            h.hypothesis.score = min(1.0, len(e.faces) / 3.0)
            self.fill_pose(h.pose.pose, e)
            d.results.append(h)
            self.fill_pose(d.bbox.center, e)
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = cd.CUBE_EDGE
            arr.detections.append(d)
        self.pub_det.publish(arr)

    @staticmethod
    def fill_pose(p, e):
        p.position.x, p.position.y, p.position.z = map(float, e.centre)
        q = e.quaternion
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)

    def publish_memory(self):
        """TF and markers of every cube seen in the last memory_s; faded once out of view."""
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        tfs = []
        for k in range(cd.N_CUBES):
            entry = self.known.get(k)
            if entry is None or now - entry[1] > self.memory_s:
                m = Marker(ns="cubes", id=k, action=Marker.DELETE)
                m.header.frame_id = self.target_frame
                arr.markers.append(m)
                self.known.pop(k, None)
                continue
            e, seen = entry
            fresh = now - seen < 0.5
            m = Marker(ns="cubes", id=k, type=Marker.CUBE, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.target_frame, stamp
            self.fill_pose(m.pose, e)
            m.scale.x = m.scale.y = m.scale.z = cd.CUBE_EDGE
            r, g, b = CUBE_COLOURS[k]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9 if fresh else 0.35
            arr.markers.append(m)
            label = Marker(ns="cube_labels", id=k, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.header = m.header
            label.pose.position.x, label.pose.position.y = m.pose.position.x, m.pose.position.y
            label.pose.position.z = m.pose.position.z + cd.CUBE_EDGE
            label.pose.orientation.w = 1.0
            label.scale.z = 0.012
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"K{k}" + ("" if fresh else f" {now - seen:.0f}s")
            arr.markers.append(label)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.target_frame, f"cube_{k}"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, e.centre)
            q = e.quaternion
            r_ = tf.transform.rotation
            r_.x, r_.y, r_.z, r_.w = map(float, q)
            tfs.append(tf)
        self.pub_markers.publish(arr)
        if tfs:
            self.tf_broadcaster.sendTransform(tfs)

    def publish_overlay(self, msg, img, detections, poses, cubes, pose):
        good = {p.marker_id for p in poses if p.reproj_px <= self.max_reproj}
        for marker_id, px in detections:
            k = cd.cube_of(marker_id)
            colour = (80, 80, 80) if k is None else tuple(int(255 * v) for v in CUBE_COLOURS[k][::-1])
            cv2.polylines(img, [px.astype(np.int32)], True, colour if marker_id in good else (0, 0, 255), 2)
            cv2.putText(img, str(marker_id), tuple(px.mean(axis=0).astype(int) - [8, -5]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
        if pose is not None:
            R_wc, t_wc = pose
            R_cw, t_cw = R_wc.T, -R_wc.T @ t_wc
            for e in cubes:
                corners_w = CUBE_CORNERS @ yaw_R(e.yaw).T + e.centre
                corners_c = corners_w @ R_cw.T + t_cw
                if np.any(corners_c[:, 2] <= 0.01):
                    continue
                px, _ = cv2.fisheye.projectPoints(corners_c.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D)
                px = px.reshape(-1, 2).astype(np.int32)
                colour = tuple(int(255 * v) for v in CUBE_COLOURS[e.cube][::-1])
                for i, j in CUBE_EDGES:
                    cv2.line(img, tuple(px[i]), tuple(px[j]), colour, 2, cv2.LINE_AA)
                top = px[np.argmin(px[:, 1])]
                text = f"K{e.cube} {e.centre[0] * 100:.1f},{e.centre[1] * 100:.1f} cm {np.degrees(e.yaw):+.0f}deg"
                cv2.putText(img, text, (int(top[0]) - 60, int(top[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, text, (int(top[0]) - 60, int(top[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            colour, 1, cv2.LINE_AA)
        out = CompressedImage(header=msg.header, format="bgr8; jpeg compressed bgr8",
                              data=cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
        self.pub_image.publish(out)


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
