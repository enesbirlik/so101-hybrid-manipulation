# so101_bringup

Launch files and configuration that start the SO-101: `robot_state_publisher`, the `controller_manager` with the hardware (mock for now), and the controllers.

```bash
ros2 launch so101_bringup so101_control.launch.py                          # mock hardware (default)
ros2 launch so101_bringup so101_control.launch.py use_mock_hardware:=false  # the servo bus, once so101_hardware exists
```

| Path | Content |
|---|---|
| `config/so101_calibration.yaml` | Joint calibration: homing offsets, ranges, zero ticks, directions (`tools/so101_calibrate.py`) |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `arm_controller` (5 joints) and `gripper_controller`. Both controllers are `JointTrajectoryController`s, position command, 100 Hz |
| `launch/so101_control.launch.py` | `robot_state_publisher` + `ros2_control_node` + controller spawners |

The robot description is `so101_description/urdf/so101.urdf.xacro` with `ros2_control:=true`. With `use_mock_hardware:=true` the joints are `mock_components/GenericSystem`, which mirrors each command to the state, starting at zero.

Planned:

- The real hardware (`so101_hardware`).
- Camera driver launch and a single "full system" launch for experiments.
