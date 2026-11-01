# so101_hardware

A C++ `ros2_control` `SystemInterface` plugin (`so101_hardware/SO101Hardware`) for the six Feetech ST3215 servos on the Waveshare bus driver board.

> **Status (2026-11-01):** builds, not yet run on the arm.

```bash
ros2 launch so101_bringup so101_control.launch.py use_mock_hardware:=false
ros2 launch so101_moveit_config demo.launch.py use_mock_hardware:=false
```

## What it does

- **Single owner of the serial port.** It opens it with an exclusive `flock`, like the Python tools, so a tool cannot open the bus while it runs, and the other way round.
- **One bus cycle per control period** (100 Hz):
  - one `SYNC READ` of position and speed of all six servos;
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

## Planned

- Load, current, temperature and voltage as state interfaces. Contact detection needs load or current, because the play hides contact from the encoders (devlog day 4).
- A torque-off / compliant mode for kinesthetic teaching.
