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

Planned:

- **Servo bring-up tool:** assign IDs one motor at a time and run small safe test moves.
