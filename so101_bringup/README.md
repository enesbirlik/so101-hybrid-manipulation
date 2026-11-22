# so101_bringup

Launch files and configuration that start the SO-101: `robot_state_publisher`, the `controller_manager` with mock hardware or the real servo bus, and the controllers.

```bash
ros2 launch so101_bringup so101_control.launch.py                          # mock hardware (default)
ros2 launch so101_bringup so101_control.launch.py use_mock_hardware:=false  # the servo bus (so101_hardware)
```

| Path | Content |
|---|---|
| `config/so101_calibration.yaml` | Joint calibration: homing offsets, ranges, zero ticks, directions (`tools/so101_calibrate.py`) |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `arm_controller` (5 joints) and `gripper_controller` (`JointTrajectoryController`s, position command, 100 Hz), `torque_controller` (the torque switch) and `gripper_torque_controller` (the gripper servo's torque limit, 0.1 % units), see `so101_hardware` |
| `launch/so101_control.launch.py` | `robot_state_publisher` + `ros2_control_node` + controller spawners |
| `scripts/so101_ready.py` | Takes the arm out of the folded rest (elbow first) so MoveIt can plan |
| `scripts/so101_lift.py` | Lifts the hand straight up (`--dz` cm, default 4), keeping the gripper orientation, without MoveIt |
| `scripts/so101_teach_check.py` | Kinesthetic teaching check: taught (torque off) vs held and replayed (torque on) poses |
| `scripts/so101_gripper_squeeze.py` | Gripper contact and squeeze characterisation: closes slowly on an object, logs angle, lag, load, current |

## Teaching by hand

The encoders read the same with the torque on or off, so one joint calibration serves both. A taught pose can still come back differently, for two mechanical reasons:
- under load the servo stops short of its goal (P gain, deadband);
- play the encoders do not see.

`so101_teach_check.py` measures the first; for the second, measure the tool with a ruler (`--pause`). Results are saved in `~/so101_data/teach_check/`.

```bash
S=install/so101_bringup/lib/so101_bringup/so101_teach_check.py
$S sag                                   # hold a pose, torque on, let go: how far each joint moves
$S record                                # torque off, Enter saves each pose
$S replay ~/so101_data/teach_check/<time>-poses.yaml --rounds 3 --pause
```

## Gripper squeeze

A position-controlled close crushes a paper cube: the servo pushes with all the torque it may use. `gripper_torque_controller` lowers the gripper servo's torque limit at runtime (`[250.0]` on `/gripper_torque_controller/commands` = 25 %, `[1000.0]` restores the configured `torque_limit`; details in `so101_hardware/README.md`).

`so101_gripper_squeeze.py` measures what limit and squeeze depth to use. With the object between the open jaws, for each torque limit it:
1. opens to the start angle (default: expected contact + 6°);
2. closes at 2°/s, logging commanded and measured angle, lag (command − measured), load, current and the applied torque limit at 50 Hz;
3. detects contact when the lag rises more than `--contact-lag-deg` (1.0) above its baseline, and takes the measured angle where the rise began;
4. commands contact −1, −2, −3, −4° (`--depths`), holds each 1.5 s, and records the mean angle, lag, load and current;
5. opens again.

On exit (also Ctrl+C) it holds the jaw where it is, then restores the torque limit. It goes through the controllers only and never opens the bus. It moves the gripper: run it on the real arm only when you mean to. Jaw model (2026-11-21): gap ≈ 13 mm + 1.35 mm/° × angle, so a 30 mm cube touches at ≈ 12.6°; commands below −10° are clamped by the hardware.

```bash
S=install/so101_bringup/lib/so101_bringup/so101_gripper_squeeze.py
$S                                          # 30 mm cube, 100 % torque limit
$S --torque-limits 1000,500,300,200         # the same at several limits
$S --object-mm 30 --speed-deg-s 1 --depths 1,2,3,4,6
```

Results: `~/so101_data/gripper/<time>-squeeze.jsonl`, one JSON object per line: `meta` (arguments), `sample` (`t`, `phase`, `cmd_deg`, `pos_deg`, `lag_deg`, `load_pct`, `current_a`, `torque_limit`, `limit_cmd`), `contact` (per limit: angle, load, current at the onset), `squeeze` (per depth), `summary`.

Tested on mock hardware only (no contact there: the mock follows its command exactly).

## Hand on the table

A hand resting on the table is, in the model, a few millimetres to over 2 cm inside it: the joint play takes up downwards and the encoders do not see it (devlog day 4; with the torque off, 22 mm measured on 2026-11-08). MoveIt refuses to plan from a start state in collision. `so101_lift.py` moves the tool point straight up first, with forward kinematics from `/robot_description` and a joint trajectory for `arm_controller`; only `shoulder_lift`, `elbow_flex` and `wrist_flex` move:

```bash
install/so101_bringup/lib/so101_bringup/so101_lift.py            # 4 cm
install/so101_bringup/lib/so101_bringup/so101_lift.py --dz 6 --dry-run
```

## Leaving the folded rest

MoveIt cannot plan from the folded rest, and rejects it as a start state:
- `shoulder_lift` lies past the planning limit;
- `elbow_flex` sits on its stop, where the forearm touches the shoulder in the model;
- the hand often rests on the table.

Once the control stack runs, take the arm out first:

```bash
install/so101_bringup/lib/so101_bringup/so101_ready.py          # elbow to 80 deg, lift into its range
install/so101_bringup/lib/so101_bringup/so101_ready.py --zero   # then elbow to 0, then the arm to 0
```

(`ros2 run so101_bringup so101_ready.py` works too, with `ros-jazzy-ros2run` installed.)

These are straight joint moves for `arm_controller`, in the order of `tools/so101_calibrate.py zero`: the elbow first, so the hand rises away from the table. The joint that moves furthest goes at 0.4 rad/s, and each move takes at least 2 s.

Tested on mock hardware from the real rest pose (lift −101.4°, elbow 91°, wrist_flex 43°). The start state was invalid before the script and valid after it.

## Robot description

The robot description is `so101_description/urdf/so101.urdf.xacro` with `ros2_control:=true`. With `use_mock_hardware:=true` the joints are `mock_components/GenericSystem`, which mirrors each command to the state, starting at zero.

With `use_mock_hardware:=false` they are `so101_hardware/SO101Hardware` on the servo bus.

Planned: a single "full system" launch for the experiments (control, camera, perception, Behavior Tree). For now `so101_moveit_config`'s `demo.launch.py camera:=true` starts control, camera and MoveIt together.

## Consistency test

`test/test_calibration_consistency.py` checks that the calibration, URDF, MoveIt limits, SRDF poses, controller list and `so101_kinematics.yaml` agree. The main point: `so101_hardware` clamps commands to the calibrated servo range, so a limit wider than that range would be clipped without a warning. It reads the source files only (no ROS, no hardware).

```bash
colcon test --packages-select so101_bringup && colcon test-result --verbose   # from ~/so101_ws
python3 -m pytest so101_bringup/test -v                                       # from the repo root
```

`elbow_flex` upper is a known, strict xfail (the folded-rest exception in `joint_limits.yaml`). If it ever passes, the test fails so the marker gets removed.
