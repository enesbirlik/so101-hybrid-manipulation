# so101_moveit_config

> **Planned: phase 4** (C++ `ros2_control` interface + MoveIt 2). Not a ROS package yet.

MoveIt 2 configuration for the SO-101, generated with the MoveIt Setup Assistant and then maintained by hand.

Planned contents:

- SRDF: an `arm` planning group (5 DOF), a `gripper` group, named poses (`home`, `pre_grasp`, …), and collision matrix.
- Kinematics solver configuration. A 5-DOF arm cannot reach arbitrary 6-D poses, so position-only or constrained IK will be evaluated.
- Joint limits kept **consistent with `so101_description`**.
- Integration with `so101_bringup` controllers for execution on real hardware.
