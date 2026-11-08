# so101_moveit_config

MoveIt 2 configuration of the SO-101, written by hand (no Setup Assistant project).

```bash
ros2 launch so101_moveit_config demo.launch.py                                       # mock hardware + move_group + RViz
ros2 launch so101_moveit_config demo.launch.py use_mock_hardware:=false camera:=true  # the real arm, with the wrist camera
```

| Launch argument | Default | Description |
|---|---|---|
| `use_mock_hardware` | `true` | `false` drives the servos through `so101_hardware` |
| `camera` | `false` | Wrist camera image, field-of-view marker and hand-eye transform (`so101_perception`) |
| `rviz` | `true` | RViz with the MotionPlanning display |
| `dashboard` | `false` | The [`so101_dashboard`](../so101_dashboard/) window |

On the real arm, unfold it first from the folded rest (`so101_bringup`'s `so101_ready.py`).

In RViz, drag the interactive marker or pick a named state (`zero`, `rest`) under *Goal State*, then *Plan & Execute*. IK is position only: with 5 joints, most 6-D poses cannot be reached, and an orientation weight made dragging fail.

- **Planner** (*Context* tab): Pilz `PTP` gives straight joint-space moves, quick and repeatable. OMPL `RRTConnect` goes around obstacles, but its paths wander more. Pilz `LIN` moves the tool in a straight line; with 5 joints, not every line is reachable.
- **Speed** (*Planning* tab): the velocity and acceleration scaling factors. The saved layout uses 0.8 and 0.5. Below about 0.3 the servos creep (commands below their deadband).

| File | Content |
|---|---|
| `config/so101.srdf` | Groups `arm` (`base_link` → `gripper_frame_link`, 5 joints) and `gripper`, named states, end effector, disabled collision pairs |
| `config/kinematics.yaml` | [pick_ik](https://github.com/PickNikRobotics/pick_ik), position only: the dragged marker sets where the gripper goes, not its orientation |
| `config/joint_limits.yaml` | Position limits from the joint calibration, 3 rad/s and 8 rad/s² for every joint, 50 % default scaling |
| `config/moveit_controllers.yaml` | `arm_controller` and `gripper_controller` (`FollowJointTrajectory`, from `so101_bringup`) |
| `config/pilz_cartesian_limits.yaml` | Cartesian limits for Pilz `LIN`/`CIRC` |
| `config/moveit.rviz` | RViz with the MotionPlanning display, wrist camera image, FOV marker and camera axes |
| `launch/demo.launch.py` | Includes `so101_bringup/launch/so101_control.launch.py` (and with `camera:=true` the wrist camera launch), starts `move_group` and RViz |

Notes:

- **Table:** the table is part of the robot description: a link `table`, fixed to `base_link`, with its top at the measured `table_z`. Plans that take the hand into the table are rejected (`GOAL_STATE_INVALID`).
- **Joint limits:** the URDF limits, cut to the calibrated ranges with a 3° margin.
  - `elbow_flex` goes up to 92.8°, because the folded rest pose reads up to 92.3° on its stop, and MoveIt rejects a start state outside the limits.
  - The URDF's own elbow limit (96.8°) lies past the real stop (91.8°).
- **Collision pairs:** only adjacent links, and a few pairs near the base, are disabled. `collisions_updater` also reports the camera and jaw against the forearm as never colliding in 20000 random poses. They stay checked, because in the hand-eye runs the camera did brush the arm.
- **Smoke test** (mock hardware, 2026-11-01):
  - Joint goals: zero, rest, gripper open/closed planned and executed within 0.01 rad.
  - IK: 10 of 10 poses taken from forward kinematics were solved.
