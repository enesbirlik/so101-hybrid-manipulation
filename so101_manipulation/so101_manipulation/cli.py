#!/usr/bin/env python3
"""Call the cube pick/place action server.

  so101_cube pick 2 [--hover-only]          grasp cube_2 (search, refine, soft grasp, lift)
  so101_cube place 25 -5 [--yaw 0]          put the held cube down, centre at (25, -5) cm in base_link
  so101_cube place --on 3                   stack the held cube on cube_3
  so101_cube open                           open the gripper slowly where the arm is

Ctrl-C cancels the running goal: the arm stops where it is and the gripper is left as it is.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger

from so101_manipulation_interfaces.action import PickCube, PlaceCube


def run_goal(node, client, goal, timeout):
    if not client.wait_for_server(timeout_sec=5.0):
        print("action server not available (is so101_manipulation pick_place running?)")
        return 2

    def on_fb(msg):
        f = msg.feedback
        g = f" | gripper {f.gripper_deg:.1f} deg" if hasattr(f, "gripper_deg") else ""
        print(f"  [{f.phase}] {f.detail}{g}")
    fut = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("goal rejected (busy?)")
        return 2
    res = gh.get_result_async()
    t0 = time.time()
    try:
        while not res.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                print("timeout: canceling")
                raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("canceling ...")
        c = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, c, timeout_sec=5.0)
        rclpy.spin_until_future_complete(node, res, timeout_sec=10.0)
    r = res.result().result if res.done() else None
    if r is None:
        print("no result")
        return 1
    print(("OK: " if r.success else f"FAILED ({r.failed_phase}): ") + r.message)
    for k in ("contact_angle_deg", "hold_angle_deg", "held_angle_deg", "wrist_roll_deg", "jaw_tilt_deg", "refine_shift_mm"):
        if hasattr(r, k) and not math.isnan(getattr(r, k)):
            print(f"  {k}: {getattr(r, k):.1f}")
    return 0 if r.success else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default="", help="namespace of the action server (default: root)")
    ap.add_argument("--timeout", type=float, default=180.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    pk = sub.add_parser("pick")
    pk.add_argument("cube", help="cube number or cube_<k>")
    pk.add_argument("--hover-only", action="store_true", help="dry run: stop at the hover")
    pl = sub.add_parser("place")
    pl.add_argument("x", type=float, nargs="?", default=float("nan"), help="cm, base_link")
    pl.add_argument("y", type=float, nargs="?", default=float("nan"), help="cm, base_link")
    pl.add_argument("--on", default="", help="stack on this cube")
    pl.add_argument("--yaw", type=float, default=float("nan"), help="cube yaw on the table, deg")
    sub.add_parser("open")
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    ns = args.server.rstrip("/")

    rclpy.init()
    node = Node("so101_cube_cli")
    try:
        if args.cmd == "pick":
            g = PickCube.Goal(cube_id=str(args.cube), hover_only=args.hover_only)
            return run_goal(node, ActionClient(node, PickCube, f"{ns}/pick_cube"), g, args.timeout)
        if args.cmd == "place":
            if not args.on and (math.isnan(args.x) or math.isnan(args.y)):
                ap.error("place needs X Y (cm) or --on K")
            g = PlaceCube.Goal(x=0.0 if math.isnan(args.x) else args.x / 100, y=0.0 if math.isnan(args.y) else args.y / 100,
                               on_cube_id=args.on, yaw_deg=args.yaw)
            return run_goal(node, ActionClient(node, PlaceCube, f"{ns}/place_cube"), g, args.timeout)
        cli = node.create_client(Trigger, "/cube_pick_place/open_gripper")
        if not cli.wait_for_service(timeout_sec=5.0):
            print("service /cube_pick_place/open_gripper not available")
            return 2
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        r = fut.result()
        print(("OK: " if r and r.success else "FAILED: ") + (r.message if r else "no answer"))
        return 0 if r and r.success else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
