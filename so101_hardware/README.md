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

Set as `<param>` in the `<hardware>` block of [`so101.ros2_control.xacro`](../so101_description/urdf/so101.ros2_control.xacro). The gains and `torque_limit` can also be set per joint (`<param>` in a `<joint>` block, through the `joint_gains` and `joint_torque_limits` tables of that file): joint value, else hardware value, else the default.

| Parameter | Default | Meaning |
|---|---|---|
| `port` | the single `/dev/serial/by-id/usb-1a86_*` | Serial port |
| `calibration_file` | `so101_bringup/config/so101_calibration.yaml` | Joint calibration |
| `torque_limit` | `1000` | Per joint or hardware level. Integer 0..1000, 0.1 % units (1000 = 100 %). Written to RAM register 48 on activation, before the torque comes on, so it is reset on every activation. Also the ceiling of the runtime `torque_limit` command (see below). 70 % cannot hold the arm horizontal: do not lower it on the arm joints. A value outside 0..1000 or not an integer stops the start-up |
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
| `torque_limit` | 0.1 % | the value last written to register 48 (only on joints that declare it: the gripper) |

The load/current/temperature/voltage four are optional. They are read when the URDF declares all four on every joint, as [`so101.ros2_control.xacro`](../so101_description/urdf/so101.ros2_control.xacro) does. `joint_state_broadcaster` publishes them on `/dynamic_joint_states`, and `so101_dashboard` shows them.

### Load and current: units and sign

- **`load`** is register 60: magnitude in 0.1 % steps (bits 0..9), direction in bit 10, published in %. It is the servo's drive output (PWM duty), so it reflects how hard the motor pushes, not a calibrated force. The sign is multiplied by the joint's calibration `direction`, so it follows the joint angle: for the gripper (`direction: 1`) a jaw pushing in the closing direction (towards smaller angles) is expected to read negative. Unverified on the arm: `so101_bringup/scripts/so101_gripper_squeeze.py` logs it raw.
- **`current`** is register 69: magnitude only (`abs`), in A, 6.5 mA steps. Always ≥ 0.
- Open point, to be settled by the squeeze run at reduced torque limits: whether 100 % load means full output or the present torque limit (the load should saturate near the limit in one case and not in the other).
- At P 16 the position lag (command − measured) was a clearer contact signal for the gripper than load (2026-11-21). On mock hardware all four read 0.

## Torque limit at runtime (grasping without crushing)

The gripper is position controlled: a close command to −10° makes the servo push with whatever torque it is allowed into the object between the jaws, enough to crush a paper cube. Register 48 (Torque Limit, RAM, u16, 0..1000 = 0..100 %) caps the motor output in the servo itself, whatever the position error.

A joint that declares a `torque_limit` command interface (the xacro does it for the gripper: `torque_limit_interface="true"` in `so101_joint`) can change it while the stack runs:

- **Command** `gripper/torque_limit`, 0.1 % units. Rounded and clamped to `0 .. torque_limit parameter` (default 1000): a runtime command can lower the limit, never raise it past the configured one. NaN, the start value, means "keep what is applied".
- **Bus timing:** in `write()`, before the goal positions, and only in a cycle where the rounded value changed: one SYNC WRITE (no reply, about 0.1 ms) of the joints that changed. Nothing extra is sent otherwise and nothing is added to `read()`. A failed write is retried next cycle.
- **State** `gripper/torque_limit`: the value last written.
- The value stays until changed, also when the controller writing it is deactivated. Activation resets it to the parameter (and the command to NaN). After a torque off/on through `torque/enable` the applied limits are sent again, as a precaution.
- `so101_bringup`'s `gripper_torque_controller` (ForwardCommandController) writes it:

```bash
ros2 topic pub --once /gripper_torque_controller/commands std_msgs/msg/Float64MultiArray "{data: [250.0]}"   # 25 % for the grasp
ros2 topic pub --once /gripper_torque_controller/commands std_msgs/msg/Float64MultiArray "{data: [1000.0]}"  # back to the parameter
```

A grasp node lowers it before the close goal and restores it after releasing. Restore it only once the jaw is no longer commanded into the object (open, or hold at the measured angle), or the full torque comes back on the object.

Why a joint command interface rather than a GPIO: the limit belongs to one servo, like the `torque_limit` parameter, and `forward_command_controller` can claim it per joint. The `torque/enable` GPIO is a GPIO because it switches all servos at once.

The value to use comes from the characterisation run (`so101_bringup/scripts/so101_gripper_squeeze.py`, `--torque-limits 1000,500,300`), not measured yet. A lower gripper `torque_limit` parameter (e.g. `joint_torque_limits = dict(gripper=500)`) would also bound the empty close against the jaw stop.

## Torque switch

The GPIO `torque/enable` (command and state: 1 on, 0 off) switches all servos' torque, for example to move the arm by hand. `so101_bringup`'s `torque_controller` writes it: `[1.0]` or `[0.0]` on `/torque_controller/commands`.

- **Off:** torque off on all servos, and no goal positions are sent. The arm drops unless it is supported.
- **On:** goal = present position, then torque on. The pose at that moment is held until the position commands come back within 2° of it. The trajectory controllers would otherwise still command the pose from before the torque went off, and the arm would jump there. Restarting `arm_controller` and `gripper_controller` makes them command the present pose; the `so101_dashboard` TORQUE ON button does that.

## Tests

```bash
colcon test --packages-select so101_hardware && colcon test-result --verbose
```

`test_servo_gains` (gain parameters) and `test_torque_limit` (torque limit parameter resolution and bounds, runtime command clamping, register 48 encoding, URDF parsing of the interfaces). No hardware needed.

## Planned

- Contact detection from load or current, because the play hides contact from the encoders (devlog day 4). The gripper squeeze log (`so101_gripper_squeeze.py`) is the first data for it.
