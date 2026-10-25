# Attribution

The robot description in this package is derived from **SO-ARM100** by TheRobotStudio and contributors.

- Source: https://github.com/TheRobotStudio/SO-ARM100
- License: [Apache License 2.0](https://github.com/TheRobotStudio/SO-ARM100/blob/main/LICENSE). This repository is also licensed under Apache-2.0; see the root [LICENSE](../LICENSE) and [NOTICE](../NOTICE).
- Source commit: [`a758567`](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101) (`main` at the time of vendoring, 2026-10-25). Last upstream change to the URDF itself: `385e8d7` (2025-07-02).
- Upstream note: the STS3215 motor parameters in the upstream model are adapted from the [Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini) project (Apache-2.0).

## Files taken from upstream

| Upstream path | Path in this package |
|---|---|
| `Simulation/SO101/so101_new_calib.urdf` | `urdf/so101_new_calib.urdf` |
| `Simulation/SO101/so101_new_calib_camera.urdf` (camera links and joints only) | `urdf/wrist_camera.xacro` |
| `Simulation/SO101/assets/*.stl` (only the meshes the URDFs reference) | `meshes/` |

## Modifications (Apache-2.0 §4(b))

- `urdf/so101_new_calib.urdf`: mesh paths changed from `assets/<file>.stl` to `package://so101_description/meshes/<file>.stl`; a header comment states the source and the change. Kinematics, inertias, and limits are unchanged.
- `urdf/wrist_camera.xacro`: the `wrist_camera_mount_link`, `wrist_camera_link`, `wrist_camera_mount_joint`, and `wrist_camera_joint` elements were extracted from `so101_new_calib_camera.urdf` and wrapped in a xacro file so they can be toggled with the `use_wrist_camera` argument. Mesh paths were changed as above.
- `urdf/so101.urdf.xacro`: new file (not from upstream) that combines the two.

<!-- Keep this list up to date whenever a vendored file is edited. -->
