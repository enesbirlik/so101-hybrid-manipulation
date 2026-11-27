#!/usr/bin/env python3
"""Plain wooden cubes/boxes (no markers, unknown sizes) in the wrist camera -> poses and measured sizes in
base_link. Drop-in replacement for cube_detector_node.py: the same topics and message convention.

Subscribes  wrist_camera/image_raw/compressed (any size; K is scaled to it)
            joint_states              frames taken while the arm moves are ignored (max_joint_speed)
Publishes   cubes/detections          vision_msgs/Detection3DArray, boxes seen in this frame
            cubes/tracked             vision_msgs/Detection3DArray, every tracked box (convention below)
            cubes/markers             visualization_msgs/MarkerArray: solid while seen, faint after
            wrist_camera/cubes/compressed  the image with the wood mask outline and fitted boxes, only while subscribed
            TF base_link -> cube_<k>  tracked pose of every box seen in the last `memory_s` seconds

Per Detection3D (both arrays):
    id, results[0].hypothesis.class_id   "cube_<k>", k = 10, 11, ... (persistent id by position: a new box
                                          or one moved more than assoc_mm gets a new id)
    results[0].hypothesis.score          detections: this frame's fit confidence; tracked: >= 0.5 exactly when stable
    results[0].pose.pose                 centre (z = bottom + h/2) and yaw folded into [-45, 45) deg
    results[0].pose.covariance           as cube_detector_node.py
    bbox.size                            MEASURED: x = edge along the yaw axis, y = the other horizontal edge,
                                         z = height (tracked: median over the track window)

Geometry in wood_detection.py.
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

import cube_detection as cd
import cube_tracking as ct
import wood_detection as wd
from cube_detector_node import CUBE_COLOURS, CUBE_EDGES, quat_to_R, stamp_s, yaw_quaternion, yaw_R

_UNIT_CORNERS = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])


class WoodCubeDetector(Node):
    def __init__(self):
        super().__init__("wood_cube_detector")
        perception = get_package_share_directory("so101_perception")
        dp = self.declare_parameter
        calib = dp("calibration_file", os.path.join(perception, "config", "wrist_camera_fisheye.yaml")).value
        self.target_frame = dp("target_frame", "base_link").value
        image_topic = dp("image_topic", "wrist_camera/image_raw/compressed").value
        joints_topic = dp("joint_states_topic", "joint_states").value
        self.memory_s = float(dp("memory_s", 30.0).value)
        w = wd.WoodParams()
        self.prm = wd.WoodParams(
            h_min=int(dp("h_min", w.h_min).value), h_max=int(dp("h_max", w.h_max).value),
            s_min=int(dp("s_min", w.s_min).value), s_max=int(dp("s_max", w.s_max).value),
            v_min=int(dp("v_min", w.v_min).value), dark_v=int(dp("dark_v", w.dark_v).value),
            dark_s_min=int(dp("dark_s_min", w.dark_s_min).value),
            x_range=tuple(dp("work_x", list(w.x_range)).value), y_range=tuple(dp("work_y", list(w.y_range)).value),
            min_iou=float(dp("min_iou", w.min_iou).value), max_rms_px=float(dp("max_rms_px", w.max_rms_px).value),
            cube_prior=float(dp("cube_prior", w.cube_prior).value))
        tp = ct.TrackerParams()
        self.tracker = wd.WoodTracker(ct.TrackerParams(
            window=int(dp("track_window", tp.window).value),
            jump_mm=float(dp("jump_mm", tp.jump_mm).value),
            jump_deg=float(dp("jump_deg", tp.jump_deg).value),
            confirm=int(dp("jump_confirm", tp.confirm).value),
            min_obs=int(dp("stable_min_obs", tp.min_obs).value),
            stable_mm=float(dp("stable_mm", tp.stable_mm).value),
            stable_deg=float(dp("stable_deg", tp.stable_deg).value)),
            assoc_mm=float(dp("assoc_mm", 20.0).value), first_id=int(dp("first_id", 10).value))
        self.max_joint_speed = float(dp("max_joint_speed", 0.1).value)
        self.settle_s = float(dp("settle_s", 0.3).value)
        table_z = dp("table_z", float("nan")).value
        self.table_z = self.table_z_from_description() if np.isnan(table_z) else float(table_z)

        with open(calib) as f:
            c = yaml.safe_load(f)
        self.K_full = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
        self.D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
        self.calib_width = c["image_width"]
        self.K, self.K_width, self.rays = None, None, None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_det = self.create_publisher(Detection3DArray, "cubes/detections", 10)
        self.pub_tracked = self.create_publisher(Detection3DArray, "cubes/tracked", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "cubes/markers", 10)
        self.pub_image = self.create_publisher(CompressedImage, "wrist_camera/cubes/compressed", qos_profile_sensor_data)
        # a fit takes ~0.1-0.3 s per box: keep only the newest frame instead of queueing stale ones
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CompressedImage, image_topic, self.on_image, latest)
        self.last_fast, self.prev_joints = None, None
        if self.max_joint_speed > 0:
            self.create_subscription(JointState, joints_topic, self.on_joints, 20)
        self.drawn = set()
        self.create_timer(0.2, self.publish_memory)
        self.get_logger().info(f"{image_topic} -> wooden cubes in {self.target_frame}; table_z {self.table_z * 1000:.1f} mm")

    def table_z_from_description(self):
        try:
            path = os.path.join(get_package_share_directory("so101_description"), "config", "so101_kinematics.yaml")
            with open(path) as f:
                return float(yaml.safe_load(f)["table_z"])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no table_z in so101_kinematics.yaml ({e}); using 0")
            return 0.0

    def on_joints(self, msg):
        t = stamp_s(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9
        speed = 0.0
        if len(msg.velocity) == len(msg.name) and msg.velocity:
            speed = float(np.max(np.abs(msg.velocity)))
        elif self.prev_joints is not None:
            t0, names0, pos0 = self.prev_joints
            dt = t - t0
            if dt > 1e-3:
                prev = dict(zip(names0, pos0))
                d = [abs(p - prev[n]) for n, p in zip(msg.name, msg.position) if n in prev]
                speed = max(d, default=0.0) / dt
        self.prev_joints = (t, list(msg.name), list(msg.position))
        if speed > self.max_joint_speed:
            self.last_fast = t

    def arm_moving(self, t_image):
        return self.last_fast is not None and t_image - self.last_fast < self.settle_s

    def camera_pose(self, frame, stamp):
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
            self.K = cd.scaled_camera(self.K_full, img.shape[1], self.calib_width)
            self.K_width = img.shape[1]
            self.rays = wd.pixel_rays(self.K, self.D, img.shape[1], img.shape[0])
        moving = self.arm_moving(stamp_s(msg.header.stamp))
        pose = None if moving else self.camera_pose(msg.header.frame_id, msg.header.stamp)
        good, bad, mask, ided = [], [], None, []
        if pose is not None:
            R_wc, t_wc = pose
            try:
                good, bad, mask = wd.detect_boxes(img, R_wc, t_wc, self.K, self.D, self.table_z, self.prm,
                                                  rays_cam=self.rays)
            except Exception as e:  # one bad frame must not take the detector down
                self.get_logger().warn(f"wood fit failed on this frame: {type(e).__name__}: {e}",
                                       throttle_duration_sec=5.0)
            ided = self.tracker.update(stamp_s(msg.header.stamp), good)
            self.publish_detections(msg.header.stamp, ided)
            self.publish_tracked()
        if self.pub_image.get_subscription_count():
            self.publish_overlay(msg, img, mask, ided, bad, pose, moving)

    @staticmethod
    def fill_pose(p, centre, yaw):
        p.position.x, p.position.y, p.position.z = map(float, centre)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = yaw_quaternion(yaw)

    def detection(self, k, centre, yaw, size, score, spread, spread_yaw):
        d = Detection3D()
        d.header.frame_id = self.target_frame
        d.id = f"cube_{k}"
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = f"cube_{k}"
        h.hypothesis.score = float(score)
        self.fill_pose(h.pose.pose, centre, yaw)
        cov = np.zeros(36)
        cov[0] = cov[7] = spread ** 2 / 2
        cov[14] = 1e-6
        cov[21] = cov[28] = 1e-4
        cov[35] = spread_yaw ** 2
        h.pose.covariance = cov.tolist()
        d.results.append(h)
        self.fill_pose(d.bbox.center, centre, yaw)
        d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = map(float, size)
        return d

    def publish_detections(self, stamp, ided):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = stamp, self.target_frame
        for k, b in ided:
            d = self.detection(k, b.centre, b.yaw, b.size, b.confidence, 0.5e-3 * (1 + b.rms_px), np.radians(3))
            d.header = arr.header
            arr.detections.append(d)
        self.pub_det.publish(arr)

    def publish_tracked(self):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = self.get_clock().now().to_msg(), self.target_frame
        for k, tr in sorted(self.tracker.tracks.items()):
            if tr.centre is None:
                continue
            size = self.tracker.size(k)
            centre = tr.centre.copy()
            spread = tr.spread if np.isfinite(tr.spread) else 1.0
            spread_yaw = tr.spread_yaw if np.isfinite(tr.spread_yaw) else 1.0
            d = self.detection(k, centre, tr.yaw, size, tr.confidence, spread, spread_yaw)
            sec = int(tr.last_stamp)
            d.header.stamp.sec, d.header.stamp.nanosec = sec, int((tr.last_stamp - sec) * 1e9)
            arr.detections.append(d)
        self.pub_tracked.publish(arr)

    def publish_memory(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()
        gone = set(self.tracker.forget(self.memory_s, now))
        arr = MarkerArray()
        tfs = []
        for k in sorted(gone | (self.drawn - set(self.tracker.tracks))):
            for ns in ("cubes", "cube_labels"):
                m = Marker(ns=ns, id=k, action=Marker.DELETE)
                m.header.frame_id = self.target_frame
                arr.markers.append(m)
            self.drawn.discard(k)
        for k, tr in sorted(self.tracker.tracks.items()):
            if tr.centre is None:
                continue
            age = tr.age(now)
            fresh = age < 0.5
            size = self.tracker.size(k)
            m = Marker(ns="cubes", id=k, type=Marker.CUBE, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.target_frame, stamp
            self.fill_pose(m.pose, tr.centre, tr.yaw)
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            r, g, b = CUBE_COLOURS[k % len(CUBE_COLOURS)]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9 if fresh else 0.35
            arr.markers.append(m)
            label = Marker(ns="cube_labels", id=k, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.header = m.header
            label.pose.position.x, label.pose.position.y = m.pose.position.x, m.pose.position.y
            label.pose.position.z = m.pose.position.z + float(size[2])
            label.pose.orientation.w = 1.0
            label.scale.z = 0.012
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"W{k} {size[0] * 1000:.0f}x{size[1] * 1000:.0f}x{size[2] * 1000:.0f}" \
                + ("" if tr.stable else " ~") + ("" if fresh else f" {age:.0f}s")
            arr.markers.append(label)
            self.drawn.add(k)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.target_frame, f"cube_{k}"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, tr.centre)
            r_ = tf.transform.rotation
            r_.x, r_.y, r_.z, r_.w = yaw_quaternion(tr.yaw)
            tfs.append(tf)
        self.pub_markers.publish(arr)
        self.publish_tracked()
        if tfs:
            self.tf_broadcaster.sendTransform(tfs)

    def publish_overlay(self, msg, img, mask, ided, bad, pose, moving):
        if mask is not None:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cnts, -1, (255, 255, 0), 1)
        for b in bad:
            cv2.polylines(img, [b.silhouette.astype(np.int32)], True, (0, 0, 255), 1)
        if moving:
            cv2.putText(img, "arm moving: frame ignored", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if pose is not None:
            R_wc, t_wc = pose
            for k, b in ided:
                corners = (_UNIT_CORNERS * b.size) @ yaw_R(b.yaw).T + b.centre
                px = wd.project(corners, R_wc, t_wc, self.K, self.D).astype(np.int32)
                colour = tuple(int(255 * v) for v in CUBE_COLOURS[k % len(CUBE_COLOURS)][::-1])
                for i, j in CUBE_EDGES:
                    cv2.line(img, tuple(px[i]), tuple(px[j]), colour, 2, cv2.LINE_AA)
                tr = self.tracker.tracks.get(k)
                mark = " *" if tr is not None and tr.stable else ""
                top = px[np.argmin(px[:, 1])]
                text = f"W{k} {b.size[0] * 1000:.0f}x{b.size[1] * 1000:.0f}x{b.size[2] * 1000:.0f} {b.iou:.2f}{mark}"
                org = (int(top[0]) - 50, int(top[1]) - 8)
                cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv2.LINE_AA)
        out = CompressedImage(header=msg.header, format="bgr8; jpeg compressed bgr8",
                              data=cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
        self.pub_image.publish(out)


def main():
    rclpy.init()
    node = WoodCubeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
