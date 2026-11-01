# so101_moveit_config

MoveIt 2 configuration of the SO-101, written by hand (no Setup Assistant project).

```bash
ros2 launch so101_moveit_config demo.launch.py   # control stack (mock hardware) + move_group + RViz
```

In RViz, drag the interactive marker or pick a named state (`zero`, `rest`) under *Goal State*, then *Plan & Execute*. Tick *Approx IK Solutions*: with 5 joints, most dragged 6-D poses cannot be reached exactly.

| File | Content |
|---|---|
| `config/so101.srdf` | Groups `arm` (`base_link` → `gripper_frame_link`, 5 joints) and `gripper`, named states, end effector, disabled collision pairs |
| `config/kinematics.yaml` | [pick_ik](https://github.com/PickNikRobotics/pick_ik): position fully, orientation half weighted |
| `config/joint_limits.yaml` | Position limits from the joint calibration, velocity/acceleration limits, 30 % default scaling |
| `config/moveit_controllers.yaml` | `arm_controller` and `gripper_controller` (`FollowJointTrajectory`, from `so101_bringup`) |
| `config/pilz_cartesian_limits.yaml` | Cartesian limits for Pilz `LIN`/`CIRC` |
| `config/moveit.rviz` | RViz with the MotionPlanning display |
| `launch/demo.launch.py` | Includes `so101_bringup/launch/so101_control.launch.py`, starts `move_group` and RViz |

Notes:

- **Table:** the table is part of the robot description: a link `table`, fixed to `base_link`, with its top at the measured `table_z`. Plans that take the hand into the table are rejected (`GOAL_STATE_INVALID`).
- **Joint limits:** the URDF limits, cut to the calibrated ranges with a 3° margin.
  - `elbow_flex` goes up to 92.8°, because the folded rest pose reads up to 92.3° on its stop, and MoveIt rejects a start state outside the limits.
  - The URDF's own elbow limit (96.8°) lies past the real stop (91.8°).
- **Collision pairs:** only adjacent links, and a few pairs near the base, are disabled. `collisions_updater` also reports the camera and jaw against the forearm as never colliding in 20000 random poses. They stay checked, because in the hand-eye runs the camera did brush the arm.
- **Smoke test** (mock hardware, 2026-11-01):
  - Joint goals: zero, rest, gripper open/closed planned and executed within 0.01 rad.
  - IK: 10 of 10 poses taken from forward kinematics were solved.
