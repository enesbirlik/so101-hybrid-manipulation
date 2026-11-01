"""Wrist camera image + the hand-eye transform gripper_link -> wrist_camera_optical_frame."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("so101_perception"), "config")
    with open(os.path.join(config, "wrist_camera_hand_eye.yaml")) as f:
        hand_eye = yaml.safe_load(f)
    t, r = hand_eye["translation"], hand_eye["rotation"]

    return LaunchDescription([
        DeclareLaunchArgument("scale", default_value="0.5", description="published image size relative to 1280x720"),
        Node(
            package="so101_perception",
            executable="wrist_camera_node.py",
            parameters=[{
                "calibration_file": os.path.join(config, "wrist_camera_fisheye.yaml"),
                "scale": LaunchConfiguration("scale"),
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", str(t["x"]), "--y", str(t["y"]), "--z", str(t["z"]),
                "--qx", str(r["x"]), "--qy", str(r["y"]), "--qz", str(r["z"]), "--qw", str(r["w"]),
                "--frame-id", hand_eye["parent_frame"], "--child-frame-id", hand_eye["child_frame"],
            ],
        ),
    ])
