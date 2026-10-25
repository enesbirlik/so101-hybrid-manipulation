# tools

Standalone helper scripts that run **without** ROS: hardware bring-up, diagnostics, and one-off utilities.

> These tools open the serial port directly. Never run them while `so101_hardware` is running; the hardware interface is the single owner of the port.

Planned:

- **Servo bring-up tool:** scan the bus, read ID, position, voltage, and temperature, assign IDs one motor at a time, and run small safe test moves. A first quick prototype proved that all six servos respond (see [Day 0](../docs/devlog/2026-10-25-day-0.md)). It is being rewritten after reviewing the Feetech protocol and existing SDKs.
- **Calibration helper (phase 3):** record per-joint zero offsets, directions, and range limits for matching the URDF.
