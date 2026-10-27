#!/usr/bin/env python3
"""Publish the real SO-101 joint angles on /joint_states (read-only phase 3 helper).

Reads every servo, converts ticks to radians with the calibration file and
publishes sensor_msgs/JointState, so RViz shows the real arm live:

  ros2 launch so101_description display.launch.py gui:=false
  python3 tools/so101_joint_states.py [--torque-off]

Torque is left as it is unless --torque-off is given. Opens the serial bus
directly: never run it while so101_hardware is running.
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import feetech_bus as fb
from feetech_bus import FeetechBus, FeetechError
from so101_calibrate import DEFAULT_FILE, JOINTS, load_calibration


class JointStatePublisher(Node):
    def __init__(self, bus, cal, rate):
        super().__init__("so101_joint_states")
        self.bus = bus
        self.cal = cal
        self.joints = [j for j in JOINTS if j.name in cal]
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in self.joints:
            c = self.cal[j.name]
            try:
                ticks = self.bus.read_position(c["id"])
            except FeetechError as e:
                self.get_logger().warn(str(e), throttle_duration_sec=1.0)
                return  # skip the whole message rather than publish a partial one
            msg.name.append(j.name)
            msg.position.append(c["direction"] * (ticks - c["zero"]) * 2 * math.pi / fb.TICKS_PER_TURN)
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the single /dev/serial/by-id/usb-1a86_* adapter)")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help=f"calibration file (default: {DEFAULT_FILE})")
    parser.add_argument("--rate", type=float, default=30.0, help="publish rate [Hz]")
    parser.add_argument("--torque-off", action="store_true", help="switch torque off first so you can move the arm")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    cal = load_calibration(args.file)
    if not cal:
        print(f"error: no calibration in {args.file}", file=sys.stderr)
        return 1

    with FeetechBus(args.port or fb.find_port()) as bus:
        if args.torque_off:
            for c in cal.values():
                bus.write_u8(c["id"], fb.TORQUE_ENABLE, 0)
        rclpy.init()
        node = JointStatePublisher(bus, cal, args.rate)
        node.get_logger().info(f"publishing {len(node.joints)} joints from {bus.port}")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
