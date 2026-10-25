# so101_hardware

> **Planned: phase 4** (C++ `ros2_control` interface + MoveIt 2). Not a ROS package yet.

A C++ `ros2_control` `SystemInterface` plugin for the six Feetech ST3215 servos on the Waveshare bus driver board.

Planned responsibilities:

- Be the **single owner of the serial port**. No other node or tool talks to the bus while the hardware interface is running.
- Read position (and later velocity, load, temperature, voltage) for all joints in one synchronous bus cycle; write position commands with sync-write.
- Convert servo ticks ↔ joint radians using the per-joint offsets and directions from phase 3 calibration.
- Expose a torque-off / compliant mode for kinesthetic teaching. Gravity compensation in PWM mode is a later experiment.
- Handle errors safely: torque off on communication loss, and clamp commands to URDF limits.
