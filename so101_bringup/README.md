# so101_bringup

> **Planned: phase 4** (C++ `ros2_control` interface + MoveIt 2). Not a ROS package yet.

Launch files and configuration that start the real robot (or a mock system).

Planned contents:

- A launch file that starts `robot_state_publisher`, `controller_manager` with `so101_hardware`, and the controllers.
- Controller configuration (`joint_state_broadcaster`, an arm trajectory controller, a gripper controller).
- A `use_mock_hardware` switch (ros2_control mock components) for development without the arm.
- Later: camera driver launch and a single "full system" launch for experiments.
