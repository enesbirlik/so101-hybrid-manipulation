#!/usr/bin/env python3
"""Characterise a grasp: close the gripper slowly on an object, find the contact, squeeze in steps.

With the object (e.g. a 30 mm paper cube) held between the open jaws, for each torque limit:
  1. open to --start-deg and set the gripper's torque limit (/gripper_torque_controller/commands);
  2. close at --speed-deg-s towards --end-deg, logging commanded angle, measured angle, lag
     (commanded - measured, > 0 = the jaw is behind the command), load and current at --rate Hz;
  3. contact = the lag rises more than --contact-lag-deg above its baseline (the lag was a
     clearer signal than load at P 16, 2026-11-21); the contact angle is the measured angle where
     the rise began;
  4. squeeze: command contact - d for each d in --depths, hold --dwell-s, record the mean
     measured angle, lag, load and current over the last --sample-s;
  5. open to --start-deg again.
The torque limit is set back to 1000 (clamped to the gripper's torque_limit parameter) at the
end, also on Ctrl+C. Jaw model (2026-11-21, valid <= 30 deg): gap = 13 mm + 1.35 mm/deg * angle,
so a 30 mm cube touches at about 12.6 deg; the empty gripper closes to about -9 deg. The hardware
clamps commands to the calibrated range (gripper >= -10 deg).

Goes through ros2_control only (gripper_controller action, gripper_torque_controller topic,
/joint_states, /dynamic_joint_states, /gripper_controller/controller_state); it never opens the
servo bus. Needs the control stack running. MOVES THE GRIPPER on the real hardware.
Output: one JSON object per line in ~/so101_data/gripper/<time>-squeeze.jsonl
(type meta | sample | contact | squeeze | summary).
"""

import argparse
import json
import math
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT = "gripper"
OUT_DIR = Path.home() / "so101_data" / "gripper"
JAW_GAP_MM_AT_0 = 13.0     # 2026-11-21 ruler fit
JAW_MM_PER_DEG = 1.35
MIN_CMD_DEG = -10.0        # calibrated range_min of the gripper
BASELINE_S = (1.5, 0.3)    # lag baseline: median over [now - 1.5 s, now - 0.3 s]
ONSET_LAG_DEG = 0.3        # the contact began where the lag was last this close to the baseline
OPEN_SPEED_DEG_S = 15.0


def expected_contact_deg(object_mm):
    return (object_mm - JAW_GAP_MM_AT_0) / JAW_MM_PER_DEG


class Squeeze(Node):
    def __init__(self):
        super().__init__("so101_gripper_squeeze")
        self.lock = threading.Lock()
        self.pos = self.ref = None          # rad
        self.load = self.current = self.limit_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 50)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_dynamic, 50)
        self.create_subscription(JointTrajectoryControllerState, "/gripper_controller/controller_state",
                                 self.on_controller_state, 50)
        self.limit_pub = self.create_publisher(Float64MultiArray, "/gripper_torque_controller/commands", 10)
        self.client = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")

    def on_joint_states(self, msg):
        if JOINT in msg.name:
            with self.lock:
                self.pos = msg.position[msg.name.index(JOINT)]

    def on_dynamic(self, msg):
        if JOINT not in msg.joint_names:
            return
        iv = msg.interface_values[msg.joint_names.index(JOINT)]
        values = dict(zip(iv.interface_names, iv.values))
        with self.lock:
            self.load = values.get("load")
            self.current = values.get("current")
            self.limit_state = values.get("torque_limit")

    def on_controller_state(self, msg):
        if JOINT in msg.joint_names and msg.reference.positions:
            with self.lock:
                self.ref = msg.reference.positions[msg.joint_names.index(JOINT)]

    def snapshot(self):
        with self.lock:
            pos, ref = self.pos, self.ref
            out = {"pos_deg": math.degrees(pos) if pos is not None else None,
                   "cmd_deg": math.degrees(ref) if ref is not None else None,
                   "load_pct": self.load, "current_a": self.current, "torque_limit": self.limit_state}
        out["lag_deg"] = out["cmd_deg"] - out["pos_deg"] if None not in (pos, ref) else None
        return out

    def wait_inputs(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.snapshot()
            if s["pos_deg"] is not None and s["load_pct"] is not None:
                return True
            time.sleep(0.1)
        return False

    def set_limit(self, value):
        self.limit_pub.publish(Float64MultiArray(data=[float(value)]))

    def send(self, deg, secs):
        """Start a move to `deg` that takes `secs` (linear in time: one point, positions only)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT]
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=[math.radians(deg)],
            time_from_start=Duration(sec=int(secs), nanosec=int(secs % 1 * 1e9))))
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("gripper_controller action server not available")
        future = self.client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.005)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f"goal to {deg:+.1f} deg rejected")
        return handle


class Logger:
    def __init__(self, path, rate):
        self.file = open(path, "w")
        self.period = 1.0 / rate
        self.t0 = time.monotonic()

    def write(self, record):
        self.file.write(json.dumps(record) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


def sample_for(node, log, secs, phase, extra, until=None):
    """Log samples for `secs` (or until `until(samples)` returns something); returns (samples, hit)."""
    samples = []
    end = time.monotonic() + secs
    next_t = time.monotonic()
    while time.monotonic() < end:
        s = node.snapshot()
        s.update(type="sample", t=round(time.monotonic() - log.t0, 4), phase=phase, **extra)
        log.write(s)
        samples.append(s)
        if until is not None:
            hit = until(samples)
            if hit is not None:
                return samples, hit
        next_t += log.period
        time.sleep(max(0.0, next_t - time.monotonic()))
    return samples, None


def move(node, log, deg, speed, phase, extra, settle=0.3):
    start = node.snapshot()["pos_deg"]
    secs = max(0.5, abs(deg - start) / speed)
    node.send(deg, secs)
    return sample_for(node, log, secs + settle, phase, extra)[0]


def fmt(value, spec, scale=1.0):
    return "n/a" if value is None else format(value * scale, spec)


def mean(samples, key):
    vals = [s[key] for s in samples if s.get(key) is not None]
    return statistics.fmean(vals) if vals else None


def contact_detector(threshold):
    def detect(samples):
        now = samples[-1]["t"]
        base = [s["lag_deg"] for s in samples
                if s["lag_deg"] is not None and now - BASELINE_S[0] <= s["t"] <= now - BASELINE_S[1]]
        if now - samples[0]["t"] < BASELINE_S[0] or not base or samples[-1]["lag_deg"] is None:
            return None
        b = statistics.median(base)
        if samples[-1]["lag_deg"] - b <= threshold:
            return None
        onset = next((s for s in reversed(samples) if s["lag_deg"] is not None and s["lag_deg"] - b <= ONSET_LAG_DEG),
                     samples[0])
        return {"baseline_lag_deg": b, "onset": onset, "detected": samples[-1]}
    return detect


def run_limit(node, log, args, limit):
    extra = {"limit_cmd": limit}
    node.set_limit(limit)
    move(node, log, args.start_deg, OPEN_SPEED_DEG_S, "open", extra, settle=args.dwell_s)
    print(f"torque limit {limit / 10:.1f} %: closing {args.start_deg:+.1f} -> {args.end_deg:+.1f} deg "
          f"at {args.speed_deg_s} deg/s")
    secs = abs(args.start_deg - args.end_deg) / args.speed_deg_s
    node.send(args.end_deg, secs)
    _, hit = sample_for(node, log, secs + 1.0, "close", extra, until=contact_detector(args.contact_lag_deg))
    result = {"limit_cmd": limit, "contact": None, "squeeze": []}
    if hit is None:
        print("  no contact detected (empty gripper, or the threshold is too high)")
        log.write({"type": "contact", "limit_cmd": limit, "found": False})
    else:
        on, det = hit["onset"], hit["detected"]
        contact = {"type": "contact", "limit_cmd": limit, "found": True, "baseline_lag_deg": hit["baseline_lag_deg"],
                   "pos_deg": on["pos_deg"], "cmd_deg": on["cmd_deg"], "load_pct": on["load_pct"],
                   "current_a": on["current_a"], "t": on["t"], "detected_at": det}
        log.write(contact)
        result["contact"] = contact
        print(f"  contact at {on['pos_deg']:+.2f} deg (cmd {on['cmd_deg']:+.2f}), load {fmt(on['load_pct'], '+.1f')} %, "
              f"current {fmt(on['current_a'], '.0f', 1000)} mA; detected {det['t'] - on['t']:.2f} s later "
              f"at {det['pos_deg']:+.2f} deg")
        for depth in args.depths:
            target = max(MIN_CMD_DEG, on["pos_deg"] - depth)
            move(node, log, target, args.speed_deg_s, f"squeeze_{depth:g}", extra, settle=0.0)
            held, _ = sample_for(node, log, args.dwell_s, f"hold_{depth:g}", extra)
            tail = [s for s in held if s["t"] >= held[-1]["t"] - args.sample_s]
            rec = {"type": "squeeze", "limit_cmd": limit, "depth_deg": depth, "cmd_deg": target,
                   **{k: mean(tail, k) for k in ("pos_deg", "lag_deg", "load_pct", "current_a", "torque_limit")}}
            log.write(rec)
            result["squeeze"].append(rec)
            print(f"  contact -{depth:g} deg: cmd {target:+.2f}, at {fmt(rec['pos_deg'], '+.2f')}, "
                  f"lag {fmt(rec['lag_deg'], '+.2f')}, load {fmt(rec['load_pct'], '+.1f')} %, "
                  f"current {fmt(rec['current_a'], '.0f', 1000)} mA")
    move(node, log, args.start_deg, OPEN_SPEED_DEG_S, "release", extra, settle=args.dwell_s)
    return result


def depths(text):
    return [float(v) for v in text.split(",") if v.strip()]


def limits(text):
    out = [int(v) for v in text.split(",") if v.strip()]
    if not all(0 <= v <= 1000 for v in out):
        raise argparse.ArgumentTypeError("torque limits are 0..1000 (0.1 %)")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--object-mm", type=float, default=30.0, help="object width, for the expected contact (default 30)")
    parser.add_argument("--start-deg", type=float, default=None,
                        help="open angle to start from (default: expected contact + 6 deg, at most 30)")
    parser.add_argument("--end-deg", type=float, default=MIN_CMD_DEG, help="close no further than this (default -10)")
    parser.add_argument("--speed-deg-s", type=float, default=2.0, help="closing speed (default 2)")
    parser.add_argument("--depths", type=depths, default=[1, 2, 3, 4], help="squeeze depths past contact, deg (default 1,2,3,4)")
    parser.add_argument("--torque-limits", type=limits, default=[1000],
                        help="gripper torque limits to test, 0.1 %% units, e.g. 1000,500,300 (default 1000)")
    parser.add_argument("--contact-lag-deg", type=float, default=1.0, help="lag rise over the baseline = contact (default 1.0)")
    parser.add_argument("--dwell-s", type=float, default=1.5, help="hold time per squeeze step (default 1.5)")
    parser.add_argument("--sample-s", type=float, default=0.5, help="averaging window at the end of a hold (default 0.5)")
    parser.add_argument("--rate", type=float, default=50.0, help="log rate, Hz (default 50)")
    parser.add_argument("--yes", action="store_true", help="do not ask before moving")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    expected = expected_contact_deg(args.object_mm)
    if args.start_deg is None:
        args.start_deg = min(30.0, expected + 6.0)
    if args.end_deg < MIN_CMD_DEG or args.end_deg >= args.start_deg or args.speed_deg_s <= 0:
        parser.error(f"need {MIN_CMD_DEG} <= --end-deg < --start-deg and --speed-deg-s > 0")

    rclpy.init()
    node = Squeeze()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / f"{datetime.now():%Y%m%d_%H%M%S}-squeeze.jsonl"
    log = None
    try:
        if not node.wait_inputs():
            node.get_logger().error("no gripper state on /joint_states and /dynamic_joint_states: is the control stack up?")
            return 1
        s = node.snapshot()
        print(f"gripper at {s['pos_deg']:+.1f} deg, torque limit {s['torque_limit']}; {args.object_mm:g} mm object "
              f"expected to touch at {expected:+.1f} deg; start {args.start_deg:+.1f}, end {args.end_deg:+.1f}, "
              f"limits {args.torque_limits}")
        if not args.yes:
            input(f"Gripper opens to {args.start_deg:+.1f} deg. Object between the open jaws, hand clear? "
                  "Enter to start, Ctrl+C to quit... ")
        log = Logger(path, args.rate)
        log.write({"type": "meta", "time": datetime.now().isoformat(), "joint": JOINT, "object_mm": args.object_mm,
                   "expected_contact_deg": expected, **{k: v for k, v in vars(args).items() if k != "yes"},
                   "jaw_model": {"gap_mm_at_0": JAW_GAP_MM_AT_0, "mm_per_deg": JAW_MM_PER_DEG}})
        results = [run_limit(node, log, args, limit) for limit in args.torque_limits]
        log.write({"type": "summary", "results": results})
        print(f"log: {path}")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        if log is not None:
            # Stop squeezing first (hold where the jaw is), only then give the torque back:
            # restoring the limit while commanded into the object would crush it.
            try:
                pos = node.snapshot()["pos_deg"]
                node.send(pos, 0.3)
                time.sleep(0.5)
            except Exception as e:  # noqa: BLE001 - the limit is restored either way
                node.get_logger().error(f"could not hold the gripper: {e}")
        node.set_limit(1000.0)   # clamped to the torque_limit parameter by the hardware
        time.sleep(0.2)
        if log is not None:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
