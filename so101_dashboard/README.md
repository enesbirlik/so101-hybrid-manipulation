# so101_dashboard

A local robot monitor for the SO-101: one window for watching autonomous runs and starting tasks. It is a C++ ROS 2 node drawn with [raylib](https://www.raylib.com/) and [Dear ImGui](https://github.com/ocornut/imgui), with no Qt and no browser.

```bash
ros2 launch so101_moveit_config demo.launch.py use_mock_hardware:=false camera:=true rviz:=false dashboard:=true
ros2 launch so101_dashboard dashboard.launch.py    # next to a stack that is already running
```

## Panels

| Panel | Shows | From |
|---|---|---|
| **3D VIEW** | The URDF posed from the joint states, the table grid, joint labels, the camera field of view. While a task runs: the plan's goal pose as a ghost and the tool path | `/robot_description`, `/joint_states`, TF, `/wrist_camera/fov`, `/display_planned_path` |
| **CAMERA** | Any `sensor_msgs/CompressedImage` topic, chosen from a list, with its frame rate | the wrist camera now; processed images later |
| **TASKS** | Task buttons, status, STOP, planner and speed settings, a log of results | `move_group` (`MoveGroup` action), the trajectory controllers |
| **JOINTS** | Per joint: position, velocity, load, current, temperature, voltage, plus a 10 s plot of one of them | `/joint_states`, `/dynamic_joint_states` |
| status bar | Link state, `/joint_states` rate, `ROS_DOMAIN_ID`, clock | |

The 3D view also checks its own forward kinematics against TF at the gripper frame (`FK/TF … mm`). It should read 0.0.

## Tasks

| Task | Does |
|---|---|
| UNFOLD | Runs `so101_bringup`'s `so101_ready.py`: out of the folded rest, elbow first |
| ZERO, REST | The SRDF named states of `arm`, planned and executed by `move_group` |
| OPEN, CLOSE | The `gripper` named states |
| STOP | Cancels the MoveIt goal and every goal on the trajectory controllers, and stops a running script: the arm holds where it is |

One task runs at a time. Named states are read from `so101_moveit_config/config/so101.srdf`. The planner (Pilz PTP or OMPL RRTConnect) and the velocity and acceleration scaling apply to the next task.

## Controls

- **3D view:** drag to orbit, right drag to pan, wheel to zoom, double-click to reset. *labels* and *scanlines* toggle the overlays.
- **F12** or `kill -USR1 <pid>` saves `so101_dashboard.png` into the working directory.
- The panels can be docked anywhere. The layout is saved in `~/.local/state/so101_dashboard/imgui.ini`; delete that file to get the default back.

## Build

The first build downloads raylib 5.5, Dear ImGui 1.92.9 (docking), rlImGui and ImPlot 1.1 at pinned commits (CMake `FetchContent`, see [`CMakeLists.txt`](CMakeLists.txt)). They need the X11 and OpenGL development headers (`libx11-dev`, `libxrandr-dev`, `libxinerama-dev`, `libxcursor-dev`, `libxi-dev`, `libgl-dev`). raylib brings its own GLFW.

```bash
cd ~/so101_ws && colcon build --packages-select so101_dashboard
```

| Path | Content |
|---|---|
| `src/main.cpp` | Window, docking layout, render loop; ROS spins on a background thread |
| `src/ros_link.*` | Subscriptions and the shared `State` (mutex-guarded) |
| `src/robot_view.*` | 3D view: URDF and STL loading, forward kinematics, rim-light shader |
| `src/camera_view.*` | Camera panel |
| `src/tasks.*` | `TaskRunner`: MoveIt goals, scripts, STOP |
| `src/panels.*` | Joints, tasks and status bar panels |
| `src/theme.*` | Colours, style, font |
| `fonts/` | [Share Tech Mono](https://fonts.google.com/specimen/Share+Tech+Mono), SIL Open Font License ([`OFL.txt`](fonts/OFL.txt)) |
