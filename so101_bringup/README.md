# so101_bringup

Launch files and configuration that start the SO-101: `robot_state_publisher`, the `controller_manager` with the hardware (mock for now), and the controllers.

```bash
ros2 launch so101_bringup so101_control.launch.py                          # mock hardware (default)
ros2 launch so101_bringup so101_control.launch.py use_mock_hardware:=false  # the servo bus (so101_hardware)
```

| Path | Content |
|---|---|
| `config/so101_calibration.yaml` | Joint calibration: homing offsets, ranges, zero ticks, directions (`tools/so101_calibrate.py`) |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `arm_controller` (5 joints) and `gripper_controller`. Both controllers are `JointTrajectoryController`s, position command, 100 Hz |
| `launch/so101_control.launch.py` | `robot_state_publisher` + `ros2_control_node` + controller spawners |
| `scripts/so101_ready.py` | Takes the arm out of the folded rest (elbow first) so MoveIt can plan |

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

Planned:

- The real hardware (`so101_hardware`).
- Camera driver launch and a single "full system" launch for experiments.
