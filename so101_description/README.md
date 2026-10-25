# so101_description

URDF/xacro description of the SO-101 follower arm with an optional wrist camera, plus an RViz display launch.

The model is derived from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (`Simulation/SO101/so101_new_calib.urdf`, Apache-2.0). See [ATTRIBUTION.md](ATTRIBUTION.md) for the exact source commit and the list of modifications.

## Usage

```bash
cd ~/so101_ws && colcon build --packages-select so101_description && source install/setup.bash

# RViz + joint sliders
ros2 launch so101_description display.launch.py
ros2 launch so101_description display.launch.py use_wrist_camera:=false

# Validate the generated URDF
xacro $(ros2 pkg prefix so101_description)/share/so101_description/urdf/so101.urdf.xacro > /tmp/so101.urdf
check_urdf /tmp/so101.urdf
```

| Launch argument | Default | Description |
|---|---|---|
| `use_wrist_camera` | `true` | Adds the wrist camera mount and `wrist_camera_link` to the gripper |
| `gui` | `true` | Starts `joint_state_publisher_gui` (sliders) |

## Files

| Path | Content |
|---|---|
| `urdf/so101_new_calib.urdf` | Vendored upstream URDF (mesh paths changed to `package://`) |
| `urdf/so101.urdf.xacro` | Entry point: includes the arm, optionally the camera |
| `urdf/wrist_camera.xacro` | Wrist camera mount + camera links (from upstream `so101_new_calib_camera.urdf`) |
| `meshes/` | STL meshes referenced by the URDF |
| `launch/display.launch.py` | `robot_state_publisher` + `joint_state_publisher_gui` + RViz |
| `rviz/display.rviz` | RViz configuration |

## Joints

Five revolute arm joints and one revolute gripper joint. Joint names follow the LeRobot convention. Limits below are copied from the upstream URDF.

| # | Joint | Parent → child link | Lower [rad] | Upper [rad] | Lower [°] | Upper [°] |
|---|---|---|---|---|---|---|
| 1 | `shoulder_pan` | `base_link` → `shoulder_link` | −1.91986 | 1.91986 | −110.0 | 110.0 |
| 2 | `shoulder_lift` | `shoulder_link` → `upper_arm_link` | −1.74533 | 1.74533 | −100.0 | 100.0 |
| 3 | `elbow_flex` | `upper_arm_link` → `lower_arm_link` | −1.69 | 1.69 | −96.8 | 96.8 |
| 4 | `wrist_flex` | `lower_arm_link` → `wrist_link` | −1.65806 | 1.65806 | −95.0 | 95.0 |
| 5 | `wrist_roll` | `wrist_link` → `gripper_link` | −2.74385 | 2.84121 | −157.2 | 162.8 |
| 6 | `gripper` | `gripper_link` → `moving_jaw_so101_v1_link` | −0.174533 | 1.74533 | −10.0 | 100.0 |

All joints use `effort="10"` and `velocity="10"` in the upstream file. These are **placeholder values**, not measured ST3215 limits.

Fixed frames: `gripper_frame_link` (tool point on `gripper_link`), plus `wrist_camera_mount_link` → `wrist_camera_link` when the camera is enabled.

**Zero convention ("new calibration"):** each joint's zero is the **middle of its range**, matching current LeRobot calibration. (The older "old calibration" convention put zero at the fully extended horizontal pose.)

## TODO: verify against real hardware

The model has **not yet been matched to the physical arm**. Until every item below is checked, treat the joint values as nominal.

- [ ] **Direction:** for each joint, a positive command in RViz turns the real joint the same way.
- [ ] **Zero offset:** URDF zero (mid-range) matches the servo position after calibration; record the per-joint offset in servo ticks.
- [ ] **Ticks ↔ radians:** confirm 4096 ticks = 2π for every ST3215 and note the sign per joint.
- [ ] **Limits:** the real mechanical range of each joint fits inside the URDF limits (watch for cable and printed-part collisions).
- [ ] **Gripper mapping:** relation between gripper joint angle and LeRobot's 0 (closed) … 100 (open) linear range.
- [ ] **Camera pose:** `wrist_camera_link` position and orientation match the real mount; the upstream camera mesh has no `scale` attribute, so check its size visually.
- [ ] **Camera frame:** add a `wrist_camera_optical_frame` (REP 103 optical convention) once fisheye intrinsics for the 135° lens are calibrated (phase 3).
- [ ] **Effort/velocity:** replace the placeholder limits with values derived from the ST3215 datasheet and measurements.
