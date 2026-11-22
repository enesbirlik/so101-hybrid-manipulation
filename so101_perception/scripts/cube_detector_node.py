#!/usr/bin/env python3
"""ArUco cubes in the wrist camera -> their poses in base_link, per frame and tracked over time.

Subscribes  wrist_camera/image_raw/compressed (any size; K is scaled to it)
            joint_states              to ignore frames taken while the arm moves (max_joint_speed)
Publishes   cubes/detections          vision_msgs/Detection3DArray, cubes seen in this frame
            cubes/tracked             vision_msgs/Detection3DArray, every tracked cube (convention below)
            cubes/markers             visualization_msgs/MarkerArray, tracked cubes: solid while seen, faint after
            wrist_camera/cubes/compressed  the image with markers and cube outlines, only while subscribed
            TF base_link -> cube_<k>  tracked pose of every cube seen in the last `memory_s` seconds

cubes/tracked, one Detection3D per cube seen in the last `memory_s`:
    id                               "cube_<k>"
    header.stamp                     time of the last image the cube was seen in (age = now - stamp)
    results[0].hypothesis.class_id   "cube_<k>"
    results[0].hypothesis.score      confidence 0..1; >= 0.5 exactly when the track is stable
    results[0].pose.pose             centre (m) and yaw (pure rotation about z, folded into [-45, 45) deg)
    results[0].pose.covariance       6x6 row-major (x y z roll pitch yaw): [0] = [7] = (spread_xy / sqrt 2)^2,
                                     [14] = small (table prior), [35] = spread_yaw^2 (rad^2), roll/pitch 1e-4
    bbox.center, bbox.size           same pose; the cube's edge (cubes.yaml)

Geometry in cube_detection.py, the filter in cube_tracking.py. A cube resting on the table has its
centre at table_z + edge/2; that height fixes the distance along the camera rays (`snap_to_table`).
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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

import cube_detection as cd
import cube_tracking as ct

# colours of the printed sheet (K0..K5), RGB 0..1
CUBE_COLOURS = [(0.96, 0.55, 0.55), (0.45, 0.70, 0.95), (0.50, 0.85, 0.50),
                (0.98, 0.75, 0.40), (0.70, 0.55, 0.95), (0.95, 0.50, 0.80)]
_UNIT_CORNERS = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
CUBE_EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_R(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def yaw_quaternion(yaw):
    return 0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        perception = get_package_share_directory("so101_perception")
        dp = self.declare_parameter
        calib = dp("calibration_file", os.path.join(perception, "config", "wrist_camera_fisheye.yaml")).value
        cubes_file = dp("cubes_file", os.path.join(perception, "config", "cubes.yaml")).value
        self.target_frame = dp("target_frame", "base_link").value
        image_topic = dp("image_topic", "wrist_camera/image_raw/compressed").value
        joints_topic = dp("joint_states_topic", "joint_states").value
        self.memory_s = float(dp("memory_s", 30.0).value)
        self.snap = bool(dp("snap_to_table", True).value)
        # per-frame fusion (cube_detection.FusionParams)
        fusion = cd.FusionParams()
        self.fusion = cd.FusionParams(
            max_reproj_px=float(dp("max_reproj_px", fusion.max_reproj_px).value),
            min_view_cos=float(dp("min_view_cos", fusion.min_view_cos).value),
            outlier_px=float(dp("outlier_px", fusion.outlier_px).value),
            outlier_mm=float(dp("outlier_mm", fusion.outlier_mm).value),
            yaw_mode=str(dp("yaw_mode", fusion.yaw_mode).value),
            top_weight=float(dp("top_weight", fusion.top_weight).value))
        refinement = dp("corner_refinement", "subpix").value
        # tracking (cube_tracking.TrackerParams)
        tp = ct.TrackerParams()
        self.tracker = ct.CubeTracker(ct.TrackerParams(
            window=int(dp("track_window", tp.window).value),
            jump_mm=float(dp("jump_mm", tp.jump_mm).value),
            jump_deg=float(dp("jump_deg", tp.jump_deg).value),
            confirm=int(dp("jump_confirm", tp.confirm).value),
            min_obs=int(dp("stable_min_obs", tp.min_obs).value),
            stable_mm=float(dp("stable_mm", tp.stable_mm).value),
            stable_deg=float(dp("stable_deg", tp.stable_deg).value)))
        # arm motion: frames within settle_s of a joint moving faster than this are ignored (<= 0: off)
        self.max_joint_speed = float(dp("max_joint_speed", 0.1).value)
        self.settle_s = float(dp("settle_s", 0.3).value)

        table_z = dp("table_z", float("nan")).value
        self.table_z = self.table_z_from_description() if np.isnan(table_z) else float(table_z)
        try:
            self.geometry = cd.load_cube_geometry(cubes_file)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cannot read {cubes_file} ({e}); nominal cubes")
            self.geometry = {None: cd.CubeGeometry()}

        with open(calib) as f:
            c = yaml.safe_load(f)
        self.K_full = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
        self.D = np.array(c["distortion_coefficients"]["data"]).reshape(4, 1)
        self.calib_width = c["image_width"]
        self.K, self.K_width = None, None

        self.detector = cd.MarkerDetector(refinement=refinement)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_det = self.create_publisher(Detection3DArray, "cubes/detections", 10)
        self.pub_tracked = self.create_publisher(Detection3DArray, "cubes/tracked", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "cubes/markers", 10)
        self.pub_image = self.create_publisher(CompressedImage, "wrist_camera/cubes/compressed", qos_profile_sensor_data)
        self.create_subscription(CompressedImage, image_topic, self.on_image, qos_profile_sensor_data)
        self.last_fast = None          # stamp (s) of the last joint state with a joint faster than the limit
        self.prev_joints = None
        if self.max_joint_speed > 0:
            self.create_subscription(JointState, joints_topic, self.on_joints, 20)
        self.create_timer(0.2, self.publish_memory)
        self.get_logger().info(
            f"{image_topic} -> cubes/detections, cubes/tracked in {self.target_frame}; table_z {self.table_z * 1000:.1f} mm"
            f"{'' if self.snap else ' (not used)'}; frames ignored while a joint moves > {self.max_joint_speed} rad/s")

    def table_z_from_description(self):
        try:
            path = os.path.join(get_package_share_directory("so101_description"), "config", "so101_kinematics.yaml")
            with open(path) as f:
                return float(yaml.safe_load(f)["table_z"])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no table_z in so101_kinematics.yaml ({e}); using 0")
            return 0.0

    def edge(self, k):
        return cd.geometry_of(self.geometry, k).edge

    # -- arm motion ---------------------------------------------------------------------
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

    # -- per frame ----------------------------------------------------------------------
    def camera_pose(self, frame, stamp):
        """target <- camera at the image time, or the latest one if the arm's TF is not there yet."""
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame, t, timeout=Duration(seconds=0.05))
                tr = tf.transform.translation
                return quat_to_R(tf.transform.rotation), np.array([tr.x, tr.y, tr.z])
            except Exception as e:  # noqa: BLE001
                self.get_logger().debug(f"TF at {t}: {e}")
                continue
        self.get_logger().warn(f"no TF {self.target_frame} <- {frame}", throttle_duration_sec=2.0)
        return None

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if self.K_width != img.shape[1]:
            self.K, self.K_width = cd.scaled_camera(self.K_full, img.shape[1], self.calib_width), img.shape[1]
        moving = self.arm_moving(stamp_s(msg.header.stamp))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        poses = cd.marker_poses(detections, self.K, self.D, self.geometry)
        pose = self.camera_pose(msg.header.frame_id, msg.header.stamp) if poses and not moving else None
        cubes = []
        if pose is not None:
            R_wc, t_wc = pose
            try:
                cubes = cd.cube_estimates(poses, R_wc, t_wc, table_z=self.table_z if self.snap else None,
                                          params=self.fusion)
            except Exception as e:  # one bad frame must not take the detector down (2026-11-22)
                self.get_logger().warn(f"cube fit failed on this frame, skipped: {type(e).__name__}: {e}",
                                       throttle_duration_sec=5.0)
                cubes = []
            t = stamp_s(msg.header.stamp)
            for e in cubes:
                self.tracker.update(e.cube, ct.Observation(t, e.centre, e.yaw, e.confidence, e.face_spread,
                                                           e.face_spread_yaw))
        if not moving:
            self.publish_detections(msg.header.stamp, cubes)
            if cubes:
                self.publish_tracked()
        if self.pub_image.get_subscription_count():
            self.publish_overlay(msg, img, detections, poses, cubes, pose, moving)

    def publish_detections(self, stamp, cubes):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = stamp, self.target_frame
        for e in cubes:
            d = Detection3D()
            d.header = arr.header
            d.id = f"cube_{e.cube}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = f"cube_{e.cube}"
            h.hypothesis.score = float(e.confidence)
            self.fill_pose(h.pose.pose, e.centre, e.yaw)
            cov = np.zeros(36)
            cov[0] = cov[7] = max(e.sigma_xy, e.face_spread) ** 2 / 2
            cov[14] = 1e-6 if e.on_table else cov[0]
            cov[21] = cov[28] = 1e-4
            cov[35] = max(e.sigma_yaw, e.face_spread_yaw) ** 2
            h.pose.covariance = cov.tolist()
            d.results.append(h)
            self.fill_pose(d.bbox.center, e.centre, e.yaw)
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = self.edge(e.cube)
            arr.detections.append(d)
        self.pub_det.publish(arr)

    def publish_tracked(self):
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = self.get_clock().now().to_msg(), self.target_frame
        for k, tr in sorted(self.tracker.tracks.items()):
            if tr.centre is None:
                continue
            d = Detection3D()
            d.header.frame_id = self.target_frame
            sec = int(tr.last_stamp)
            d.header.stamp.sec, d.header.stamp.nanosec = sec, int((tr.last_stamp - sec) * 1e9)
            d.id = f"cube_{k}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = f"cube_{k}"
            h.hypothesis.score = float(tr.confidence)
            self.fill_pose(h.pose.pose, tr.centre, tr.yaw)
            spread = tr.spread if np.isfinite(tr.spread) else 1.0
            spread_yaw = tr.spread_yaw if np.isfinite(tr.spread_yaw) else 1.0
            cov = np.zeros(36)
            cov[0] = cov[7] = spread ** 2 / 2
            cov[14] = 1e-6
            cov[21] = cov[28] = 1e-4
            cov[35] = spread_yaw ** 2
            h.pose.covariance = cov.tolist()
            d.results.append(h)
            self.fill_pose(d.bbox.center, tr.centre, tr.yaw)
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = self.edge(k)
            arr.detections.append(d)
        self.pub_tracked.publish(arr)

    @staticmethod
    def fill_pose(p, centre, yaw):
        p.position.x, p.position.y, p.position.z = map(float, centre)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = yaw_quaternion(yaw)

    # -- memory: markers, TF, tracked ---------------------------------------------------
    def publish_memory(self):
        """Markers and TF of every cube tracked in the last memory_s; faded once out of view."""
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()
        gone = set(self.tracker.forget(self.memory_s, now))
        arr = MarkerArray()
        tfs = []
        for k in range(cd.N_CUBES):
            tr = self.tracker.tracks.get(k)
            if tr is None or tr.centre is None:
                if k in gone or tr is None:
                    m = Marker(ns="cubes", id=k, action=Marker.DELETE)
                    m.header.frame_id = self.target_frame
                    arr.markers.append(m)
                continue
            age = tr.age(now)
            fresh = age < 0.5
            edge = self.edge(k)
            m = Marker(ns="cubes", id=k, type=Marker.CUBE, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.target_frame, stamp
            self.fill_pose(m.pose, tr.centre, tr.yaw)
            m.scale.x = m.scale.y = m.scale.z = edge
            r, g, b = CUBE_COLOURS[k]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9 if fresh else 0.35
            arr.markers.append(m)
            label = Marker(ns="cube_labels", id=k, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.header = m.header
            label.pose.position.x, label.pose.position.y = m.pose.position.x, m.pose.position.y
            label.pose.position.z = m.pose.position.z + edge
            label.pose.orientation.w = 1.0
            label.scale.z = 0.012
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"K{k}" + ("" if tr.stable else " ~") + ("" if fresh else f" {age:.0f}s")
            arr.markers.append(label)
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

    def publish_overlay(self, msg, img, detections, poses, cubes, pose, moving):
        good = {p.marker_id for p in poses if p.reproj_px <= self.fusion.max_reproj_px}
        used = {i for e in cubes for i in e.faces}
        for marker_id, px in detections:
            k = cd.cube_of(marker_id)
            colour = (80, 80, 80) if k is None else tuple(int(255 * v) for v in CUBE_COLOURS[k][::-1])
            if marker_id in good and cubes and marker_id not in used:
                colour = (0, 165, 255)     # seen, but dropped as an outlier
            cv2.polylines(img, [px.astype(np.int32)], True, colour if marker_id in good else (0, 0, 255), 2)
            cv2.putText(img, str(marker_id), tuple(px.mean(axis=0).astype(int) - [8, -5]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
        if moving:
            cv2.putText(img, "arm moving: frame ignored", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                        cv2.LINE_AA)
        if pose is not None:
            R_wc, t_wc = pose
            R_cw, t_cw = R_wc.T, -R_wc.T @ t_wc
            for e in cubes:
                corners_w = (_UNIT_CORNERS * self.edge(e.cube)) @ yaw_R(e.yaw).T + e.centre
                corners_c = corners_w @ R_cw.T + t_cw
                if np.any(corners_c[:, 2] <= 0.01):
                    continue
                px, _ = cv2.fisheye.projectPoints(corners_c.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D)
                px = px.reshape(-1, 2).astype(np.int32)
                colour = tuple(int(255 * v) for v in CUBE_COLOURS[e.cube][::-1])
                for i, j in CUBE_EDGES:
                    cv2.line(img, tuple(px[i]), tuple(px[j]), colour, 2, cv2.LINE_AA)
                top = px[np.argmin(px[:, 1])]
                tr = self.tracker.tracks.get(e.cube)
                mark = "" if tr is None or not tr.stable else " *"
                text = (f"K{e.cube} {e.centre[0] * 100:.1f},{e.centre[1] * 100:.1f} cm {np.degrees(e.yaw):+.0f}deg "
                        f"{e.confidence:.2f}{mark}")
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
