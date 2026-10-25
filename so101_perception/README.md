# so101_perception

> **Planned: phase 3** (fisheye camera calibration) and **phase 5** (ArUco + object detection → classical pick & place). Not a ROS package yet.

Perception for the 135° wrist-mounted USB camera.

Planned contents:

- **Phase 3:** fisheye intrinsic calibration (the wide lens needs a fisheye/equidistant model, not plain pinhole), plus a check of the camera-to-gripper extrinsics against the URDF.
- **Phase 5:** ArUco marker detection for workspace and target poses; object detection giving target poses in the robot frame; outputs consumed by the Behavior Tree.
- Confidence / uncertainty outputs, which are needed for the confidence-based handover trigger studied in phase 7.
