# so101_perception

Perception for the 135° wrist-mounted USB camera.

## Wrist camera

```bash
ros2 launch so101_perception wrist_camera.launch.py [scale:=0.5]
ros2 launch so101_moveit_config demo.launch.py camera:=true   # the same, inside the MoveIt demo
```

- `scripts/wrist_camera_node.py` publishes at 15 fps:

  | Topic | Content |
  |---|---|
  | `/wrist_camera/image_raw` | Raw image |
  | `/wrist_camera/image_raw/compressed` | JPEG of the same image, ~27 kB |
  | `/wrist_camera/camera_info` | Calibration at the published size |
  | `/wrist_camera/fov` | The field of view as a marker (1 Hz): rays through the image border from the fisheye model, 15 cm long; 135° across the diagonal |

  - The camera is mounted upside down. Every frame is turned by 180°, as in `tools/so101_camera_calib.py`, so the image and its calibration are those of the upright image.
  - `scale` shrinks the image and its camera matrix. The default 0.5 gives 640×360, which is enough for RViz.
- **Light on the CPU** (5–7 % of one core while viewed, ~0 otherwise; the first version took 11–31 %):
  - A thread takes every MJPG frame undecoded, at the camera's 25 fps.
  - libjpeg decodes it straight at the published size.
  - Frames are decoded only while someone subscribes.
- **Known limits of the raw topic:**
  - At 690 kB per frame, best-effort DDS drops part of it (~12 of 15 Hz arrive). RViz therefore shows the compressed topic.
  - `CAP_PROP_BUFFERSIZE 1` halved the camera's own rate (25 → 13 fps), so it is not used.
- The launch file also publishes the hand-eye result as a static transform, `gripper_link` → `wrist_camera_optical_frame`.

| File | Content |
|---|---|
| `config/wrist_camera_fisheye.yaml` | Intrinsics, equidistant (fisheye) model, 1280×720 ([tools](../tools/README.md#so101_camera_calibpy)) |
| `config/wrist_camera_hand_eye.yaml` | `gripper_link` → `wrist_camera_optical_frame`, from the kinematic calibration |

## ArUco cubes

```bash
ros2 launch so101_perception wrist_camera.launch.py cubes:=true
ros2 launch so101_moveit_config demo.launch.py camera:=true cubes:=true
```

The cubes: 30 mm edge, one `DICT_4X4_50` marker per face (black square 22 mm), cube *k* (0–5) carries ids 6*k* … 6*k*+5. `scripts/cube_detector_node.py` finds them in the wrist image and gives their pose in `base_link`:

| Topic | Content |
|---|---|
| `/cubes/detections` | `vision_msgs/Detection3DArray`: the cubes seen in this frame (`id` = `cube_<k>`, score = faces seen / 3) |
| `/cubes/markers` | Boxes and labels for RViz and the dashboard: solid while seen, faint for up to `memory_s` (30 s) after |
| `/wrist_camera/cubes/compressed` | The image with the markers, cube outlines and positions drawn in; only while someone watches |
| TF `base_link` → `cube_<k>` | Every cube seen in the last `memory_s` |

- **How:**
  1. The marker corners are undistorted with the fisheye model, and each marker's pose comes from `solvePnP` (IPPE_SQUARE).
  2. The cube centre lies 15 mm behind each face.
  3. The cube's rotation about the vertical comes from the face's own axes, modulo 90°. The face layout on the cube does not matter, and one face is enough.
  4. Faces seen at a grazing angle (< 0.35 cosine) are dropped; the rest are weighted by that angle.
- **Table prior (`snap_to_table`, on by default):** the depth of a small marker is its weakest part (a few mm at 25 cm), while the ray direction is good. A cube resting on the table has its centre at `table_z` + 15 mm (`table_z` from `so101_description/config/so101_kinematics.yaml`). So every face's estimate is slid along its camera ray to that height, if it is within 20 mm of it. A stacked or held cube is left alone.
- The geometry is in `scripts/cube_detection.py` (no ROS). `test/test_cube_detection.py` ray-traces a cube through the fisheye model and checks it: ≤ 1.2 mm and < 1° with the table prior, ≤ 5 mm without.
- The absolute accuracy also depends on the hand-eye transform and the joint origins, which were fitted before the shoulder zero changed (2026-11-14): refit them before trusting millimetres.
- For small or far cubes, run the camera at `scale:=1.0`: at 640×360 a marker 30 cm away is only ~27 px wide.

## Planned

- **Phase 5:** object detection beyond markers; outputs consumed by the Behavior Tree.
- Confidence / uncertainty outputs, which are needed for the confidence-based handover trigger studied in phase 7.
