# tools

Standalone helper scripts that run **without** ROS: hardware bring-up, diagnostics, and one-off utilities.

> These tools open the serial port directly. Never run them while `so101_hardware` is running; the hardware interface is the single owner of the port.

The port defaults to the single `/dev/serial/by-id/usb-1a86_*` device (the Waveshare board's CH343). Nothing else is opened or probed, and the port is opened with an exclusive lock.

## `feetech_bus.py`

Minimal Feetech STS (ST3215) protocol driver on pyserial: ping, read/write registers, EEPROM unlock/lock, sign-magnitude decoding.

## `so101_calibrate.py`

```bash
python3 tools/so101_calibrate.py scan                  # ping servos, registers, voltage, temperature
python3 tools/so101_calibrate.py live [--torque-off]   # live encoder ticks (+ deg/rad once calibrated)
python3 tools/so101_calibrate.py calibrate [--joints elbow_flex gripper]
python3 tools/so101_calibrate.py check                 # startup sanity check, exit code 1 on mismatch
python3 tools/so101_calibrate.py zero                  # slow move to the calibrated zero pose, then hold
python3 tools/so101_calibrate.py off                   # torque off on every joint (support the arm)
```

`zero` moves all joints together (slowest ~18°/s, 70 % torque limit). If a joint stops getting closer to its goal for 0.5 s, every joint freezes in place and the stuck joint is reported.

`calibrate` steps:

Joint by joint, starting with every joint free (torque off):

1. Move the joint by hand to both ends of its range. It is tracked live across the 0/4095 encoder wrap. The display shows where both ends would fall relative to the current pose, next to the URDF limits.
2. Hold the joint at the pose that should be its URDF zero and press Enter, then confirm.
3. The homing offset (EEPROM, address 31) is shifted so this zero pose reads ~2048. The measured range (2° margin) becomes the servo's own position limits. Readback is verified, and the old values are restored on a mismatch.
4. The joint is saved to the calibration file and locked (torque on) before the next joint starts. The live display shows the load on every locked joint.

Ctrl+C switches torque off everywhere. Joints already written stay saved; the joint in progress is unchanged.

Output: [`so101_bringup/config/so101_calibration.yaml`](../so101_bringup/config/so101_calibration.yaml) with `homing_offset`, `range_min`, `range_max`, `zero` and `direction` per joint:

```
joint_rad = direction * (ticks - zero) * 2*pi / 4096
```

The ST3215 encoder is absolute within one turn and the offset lives in EEPROM, so calibration is a one-time step. Before each start, `check` (and later `so101_hardware`) only verifies that offsets and positions still match. Recalibrate after a horn was removed or a servo was swapped.

## `so101_joint_states.py`

Publishes the real joint angles on `/joint_states` (rclpy, 30 Hz, read-only) using the calibration file, so RViz shows the real arm live:

```bash
ros2 launch so101_description display.launch.py gui:=false
python3 tools/so101_joint_states.py [--torque-off]
```

## `so101_teleop.py`

Keyboard joint teleop, no ROS or MoveIt. `q/a w/s e/d r/f t/g y/h` jog the six joints, `z` goes to zero (shoulder_lift first, then elbow_flex, then the rest, so the folded arm does not push itself into the table), `space` stops, `+/-` change the step, `x` quits. Goals stay inside the calibrated range and never run more than 8° ahead of the joint. A watchdog freezes the arm and prints torque enable, load, current, voltage, temperature and status when a joint falls 15° behind its goal for 0.5 s.

```bash
python3 tools/so101_teleop.py
```

Planned:

- **Servo bring-up tool:** assign IDs one motor at a time and run small safe test moves.

## `so101_camera_calib.py`

Fisheye (equidistant) intrinsic calibration of the 135° wrist camera, 1280x720 MJPG (the 16:9 modes show the whole sensor centred; every 4:3 mode is an off-centre crop of it), camera mounted upside down so frames are rotated 180° as read, with the printed calibrx.io 7x5 ChArUco board (DICT_6X6_250). The print came out at 96 % (50 mm bar = 48 mm), so the squares are 24 mm: pass `--square-mm` if you reprint.

```bash
python3 tools/so101_camera_calib.py capture     # SPACE saves a frame (>= 8 of 24 corners), q quits
python3 tools/so101_camera_calib.py calibrate   # cv2.fisheye, drops outlier frames, writes the YAML
python3 tools/so101_camera_calib.py view [--balance 0.5]   # raw | undistorted live check
```

Frames go to `~/so101_data/wrist_calib_1280x720/`. The result goes to [`so101_perception/config/wrist_camera_fisheye.yaml`](../so101_perception/config/wrist_camera_fisheye.yaml) in ROS `camera_info` format (`distortion_model: equidistant`, k1..k4).

## `so101_hand_eye.py`

Eye-in-hand calibration: the wrist camera pose relative to `gripper_link`. The ChArUco board lies fixed on the table. The arm is moved by hand, torque off. At each pose the tool reads the joint ticks, computes the gripper pose with forward kinematics straight from `so101_new_calib.urdf` (no ROS needed), and finds the board pose from the fisheye-undistorted corners.

```bash
python3 tools/so101_hand_eye.py capture [--torque-off]   # by hand, torque off: SPACE saves when the arm is still
python3 tools/so101_hand_eye.py auto [--poses 20]        # torque on: set a centre pose, the arm visits poses around it
python3 tools/so101_hand_eye.py solve [--refine-joints]  # all five cv2.calibrateHandEye methods, keeps the most consistent
```

Prefer `auto`. With torque off, gravity and the hand move the joints inside their play. On the first try the board spread was 15 mm / 3.4° by hand and 3.1 mm / 1.2° with `auto`. `auto` needs one rough `capture` + `solve` result to keep the board in view. Its poses stay inside the calibrated ranges, at least 5 cm above the table, and outside the jerk zones predicted in the day-1 devlog. Each pose is reached from both sides, and `solve` reports how far apart the servos stop (about 2°, which the encoders measure) and how much the camera sees beyond that (about 0.2°).

`--refine-joints` fits the camera pose and the zero offsets of `shoulder_lift`, `elbow_flex` and `wrist_flex` together, on the corner reprojection error. The board is pinned to the table plane the arm stands on (z = 0, horizontal, only x, y and yaw free). With a free board pose, a shoulder error is absorbed by tilting the board. The fit is also checked on held-out poses and per run (centre pose). It only reports and does not change the joint calibration. Apply an offset only when every run agrees on it. `solve` also prints where the board sits relative to `base_link`; on the table it should be at ~0 mm and ~0°.

Result on 2026-10-28 (114 samples from 3 runs): the `shoulder_lift` zero was off by +10.5° and is now corrected in the calibration file (2048 → 1929). After that, the board spread is 4.7 mm / 0.71° and the camera sits at (−1.2, 69.5, −5.1) mm in `gripper_link`. Recalibrating `shoulder_lift` with `so101_calibrate.py` would undo this correction. See [devlog day 3](../docs/devlog/2026-10-28-day-3.md).

`solve` checks every method by where it puts the (fixed) board in `base_link`; the spread across samples is the quality measure. Samples (`~/so101_data/hand_eye/`) store raw ticks and images, so `solve` can be rerun after a joint or camera recalibration. Result: [`so101_perception/config/wrist_camera_hand_eye.yaml`](../so101_perception/config/wrist_camera_hand_eye.yaml), `gripper_link` → `wrist_camera_optical_frame` (OpenCV optical frame of the upright image).
