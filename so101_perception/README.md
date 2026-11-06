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

## Planned

- **Phase 5:** ArUco marker detection for workspace and target poses; object detection giving target poses in the robot frame; outputs consumed by the Behavior Tree.
- Confidence / uncertainty outputs, which are needed for the confidence-based handover trigger studied in phase 7.
