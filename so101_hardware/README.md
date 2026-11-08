# so101_hardware

A C++ `ros2_control` `SystemInterface` plugin (`so101_hardware/SO101Hardware`) for the six Feetech ST3215 servos on the Waveshare bus driver board.

> **Status (2026-11-06):** runs the arm under MoveIt. `/joint_states` at 98 Hz, no bus errors. Fixed on the way:
> - Slow moves crept and shook (commands below the servo deadband per cycle). Faster MoveIt limits fixed it; P stays 16.
> - From the folded rest, the start state is in collision with the model. `so101_bringup`'s `so101_ready.py` unfolds the arm first.
> - Between plan and execute, `shoulder_lift` drifted more than 0.05 rad. The start tolerance is now 0.1 rad.

```bash
ros2 launch so101_bringup so101_control.launch.py use_mock_hardware:=false
ros2 launch so101_moveit_config demo.launch.py use_mock_hardware:=false
```

## What it does

- **Single owner of the serial port.** It opens it with an exclusive `flock`, like the Python tools, so a tool cannot open the bus while it runs, and the other way round.
- **One bus cycle per control period** (100 Hz):
  - one `SYNC READ` of position, speed, load, voltage, temperature and current of all six servos;
  - one `SYNC WRITE` of the goal positions.
- **Ticks ↔ radians** from [`so101_bringup/config/so101_calibration.yaml`](../so101_bringup/config/so101_calibration.yaml): `joint_rad = direction * (ticks - zero) * 2π / 4096`. Commands are clamped to the calibrated range in ticks.
- **Startup check** (`on_configure`): every servo answers and still has the homing offset of the calibration. Otherwise it refuses to start.
- **Activation:** goal = present position before the torque comes on, and the commands start at the present position, so the arm holds where it is.
- **Deactivation:** the torque stays on. Switching it off would drop the arm. Release it with `python3 tools/so101_calibrate.py off` while supporting the arm.
- **Missed replies:** a missed reply keeps the last state. After `max_bus_failures` failed reads in a row, `read()` reports an error.

## Parameters

Set as `<param>` in the `<hardware>` block of [`so101.ros2_control.xacro`](../so101_description/urdf/so101.ros2_control.xacro).

| Parameter | Default | Meaning |
|---|---|---|
| `port` | the single `/dev/serial/by-id/usb-1a86_*` | Serial port |
| `calibration_file` | `so101_bringup/config/so101_calibration.yaml` | Joint calibration |
| `torque_limit` | `1000` | 0.1 % units. 70 % cannot hold the arm horizontal |
| `acceleration` | `0` | 100 step/s² units, 0 = no servo-side limit (the trajectory controller shapes the motion) |
| `goal_speed` | `0` | step/s, 0 = no servo-side limit |
| `max_bus_failures` | `5` | Consecutive failed reads before an error |
| `p_coefficient`, `d_coefficient`, `i_coefficient` | `16`, `32`, `0` | Position loop gains, LeRobot's values for the SO-100/101. The factory P of 32 made the arm shake under MoveIt. They are written on activation with the EEPROM lock on, so they only change RAM: the factory values return after a power cycle |

## State interfaces

Per joint, from one SYNC READ of addresses 56 to 70:

| Interface | Unit | Register |
|---|---|---|
| `position` | rad | 56, through the joint calibration |
| `velocity` | rad/s | 58 |
| `load` | % of the torque limit, signed like the joint | 60 |
| `current` | A | 69 (6.5 mA steps) |
| `temperature` | °C | 63 |
| `voltage` | V | 62 |

The last four are optional. They are read when the URDF declares all four on every joint, as [`so101.ros2_control.xacro`](../so101_description/urdf/so101.ros2_control.xacro) does. `joint_state_broadcaster` publishes them on `/dynamic_joint_states`, and `so101_dashboard` shows them.

## Torque switch

The GPIO `torque/enable` (command and state: 1 on, 0 off) switches all servos' torque, for example to move the arm by hand. `so101_bringup`'s `torque_controller` writes it: `[1.0]` or `[0.0]` on `/torque_controller/commands`.

- **Off:** torque off on all servos, and no goal positions are sent. The arm drops unless it is supported.
- **On:** goal = present position, then torque on. The pose at that moment is held until the position commands come back within 2° of it. The trajectory controllers would otherwise still command the pose from before the torque went off, and the arm would jump there. Restarting `arm_controller` and `gripper_controller` makes them command the present pose; the `so101_dashboard` TORQUE ON button does that.

## Planned

- Contact detection from load or current, because the play hides contact from the encoders (devlog day 4).
