# so101_policy

> **Planned: phase 6** (demonstration data + VLA policy). Not a ROS package yet.

Learned policy for the precise phase of the task.

Planned contents:

- Demonstration recording. There is no leader arm, so demonstrations will come from kinesthetic teaching (torque-off / gravity-compensated arm moved by hand), with joint states and wrist camera frames recorded.
- Conversion to a LeRobot-compatible dataset format.
- Fine-tuning a VLA / visuomotor policy and running it as a ROS 2 node that sends commands through `ros2_control`.
- Datasets and checkpoints are **not** committed to this repo (see `.gitignore`); they will be linked externally.
