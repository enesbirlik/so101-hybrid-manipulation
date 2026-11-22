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

The cubes: 30 mm edge, one `DICT_4X4_50` marker per face (black square 22 mm), cube *k* (0–5) carries ids 6*k* … 6*k*+5. They are folded from paper, so they are not exact (see below). `scripts/cube_detector_node.py` finds them in the wrist image and gives their pose in `base_link`:

| Topic | Content |
|---|---|
| `/cubes/detections` | `vision_msgs/Detection3DArray`: the cubes seen in this frame (`id` = `cube_<k>`, score = this frame's confidence, covariance from the fit and the face spread) |
| `/cubes/tracked` | `vision_msgs/Detection3DArray`: every cube tracked over the last frames, for the grasp (convention below) |
| `/cubes/markers` | Boxes and labels of the tracked cubes for RViz and the dashboard: solid while seen, faint for up to `memory_s` (30 s) after; the label gets a `~` while the track is not stable |
| `/wrist_camera/cubes/compressed` | The image with the markers, cube outlines, positions, confidence (`*` = stable) drawn in; markers dropped as outliers in orange; only while someone watches |
| TF `base_link` → `cube_<k>` | Tracked pose of every cube seen in the last `memory_s` |

### `/cubes/tracked` for the grasp

One `Detection3D` per cube seen in the last `memory_s`, in `base_link`:

| Field | Meaning |
|---|---|
| `id`, `results[0].hypothesis.class_id` | `cube_<k>` |
| `header.stamp` | time of the last image the cube was seen in: age = now − stamp |
| `results[0].hypothesis.score` | confidence 0…1. **≥ 0.5 exactly when the track is stable** (enough agreeing observations, small spread, no sign of a move); within each half it grows with the observations, their quality and a smaller spread |
| `results[0].pose.pose` | centre (m) and yaw: a pure rotation about z, folded into [−45°, 45°) (the cube looks the same every 90°) |
| `results[0].pose.covariance` | 6×6 row-major (x y z roll pitch yaw): `[0]` = `[7]` = spread_xy² / 2, `[35]` = spread_yaw² (rad²); z, roll and pitch are fixed by the table prior (small values) |
| `bbox.center`, `bbox.size` | the same pose; the cube's edge from `config/cubes.yaml` |

```python
k = int(d.id.split("_")[1]); h = d.results[0]
stable = h.hypothesis.score >= 0.5
age = now - Time.from_msg(d.header.stamp).nanoseconds * 1e-9
spread_xy = math.sqrt(2 * h.pose.covariance[0]); spread_yaw = math.sqrt(h.pose.covariance[35])
yaw = 2 * math.atan2(h.pose.pose.orientation.z, h.pose.pose.orientation.w)
```

The spread is the consistency of the observations (how much they scatter, plus the faces' disagreement), not the absolute accuracy: that also depends on the hand-eye transform and the joint origins (see the last point below).

### How

Per frame (`scripts/cube_detection.py`, no ROS):

1. The marker corners are undistorted with the fisheye model. Every marker gets **both** IPPE_SQUARE poses: a small marker seen obliquely has two plausible tilts, and the one with the lower error can be the wrong one. On the real rest-pose frames the top faces of two cubes came out pointing down that way; the old detector put those cubes 18 and 60 mm off.
2. A cube resting on the table stands upright, so only its yaw (mod 90°) is free. The face poses propose yaw hypotheses. For each, every marker is put on a distinct face of the upright cube (top or a side, and its quarter turn on it), using whichever IPPE pose fits.
3. **One rigid upright cube is fitted to the corner rays of all its faces at once** (x, y, yaw; z too when it is not on the table), Gauss-Newton with Huber weights, residuals in pixels. Faces are weighted by view cosine and marker size. The hypothesis and assignment with the lowest cost win.
4. **Outlier faces are dropped**: a face whose own residual is > 2 px (and 3× the median), or, with three faces, whose face-implied centre is > 5 mm from the median of the others.
5. Every face is also fitted alone. The spread of these face-implied centres and yaws is the **face spread**, and goes into the confidence.
6. **Yaw**: the mean (mod 90°) of the face-implied yaws, the top face weighted double. The top face's marker edges lie in a horizontal plane and give the rotation about the vertical directly; a side face gives it only through its out-of-plane tilt. Using the top face alone was worse: its own printing error then goes straight into the yaw.
7. **Confidence** (per frame, 0…1) = (1 − 0.5ⁿ) for n faces × exp(−rms²/2) for the fit residual in px × min(1, marker side / 20 px) × exp(−(face spread / 3 mm)²/2) × 0.75 without a well-seen top face.

**Table prior (`snap_to_table`, on by default):** the depth of a small marker is its weakest part, while the ray direction is good. A cube resting on the table has its centre at `table_z` + edge/2 (`table_z` from `so101_description/config/so101_kinematics.yaml`). That height is fixed in the fit when the free fit puts the cube within 20 mm of it. A stacked or held cube is fitted with a free height. A cube that does not stand upright (tipped over) gets the plain average of its face poses and confidence 0.1.

Over time (`scripts/cube_tracking.py`, no ROS): per cube the last 20 observations in `base_link` (the camera moves with the arm; a cube that lies still does not). Centre = weighted median, yaw = weighted circular mean mod 90°. An observation more than 10 mm / 12° away is held back. Three agreeing ones in a row mean the cube was moved, and the track restarts from them; a single stray frame is dropped. **Stable** = at least 5 observations, spread ≤ 2 mm and ≤ 3°, nothing held back. Frames taken while a joint moves faster than `max_joint_speed`, or within `settle_s` after, are ignored (`/joint_states`; velocities, or position differences when there are none): the image and the TF of a moving arm do not match, and the image is blurred.

### Parameters

| Parameter | Default | |
|---|---|---|
| `cubes_file` | `config/cubes.yaml` | edge and marker size, per cube if needed |
| `table_z`, `snap_to_table` | from `so101_kinematics.yaml`, `true` | table prior |
| `max_reproj_px`, `min_view_cos` | 3.0, 0.2 | a marker's IPPE error and view cosine for it to be used |
| `outlier_px`, `outlier_mm` | 2.0, 5.0 | face outlier limits |
| `yaw_mode`, `top_weight` | `blend`, 2.0 | `blend`, `top` (the top face alone when seen) or `fused` (the joint fit's yaw) |
| `corner_refinement` | `subpix` | `subpix`, `contour`, `apriltag`, `none` (subpix gave the lowest face spread on the real frames) |
| `track_window`, `jump_mm`, `jump_deg`, `jump_confirm` | 20, 10, 12, 3 | tracker |
| `stable_min_obs`, `stable_mm`, `stable_deg` | 5, 2.0, 3.0 | stable flag |
| `max_joint_speed`, `settle_s`, `joint_states_topic` | 0.1 rad/s, 0.3 s, `joint_states` | motion gate; `max_joint_speed` ≤ 0 turns it off |
| `memory_s` | 30 | tracks, markers and TF of cubes not seen for this long are dropped |

### Paper cubes: accuracy

`test/cube_sim.py` ray-traces imperfect paper cubes through the fisheye model: every face tilted up to 4°, shifted ±0.5 mm, bulged up to 1.5 mm; the marker rotated ±3°, off-centre ±2 mm per axis, scaled ±2 %; the cube ±2 % in size and tilted up to 2°. The image is supersampled, blurred, noised and JPEG-compressed at 640×360. Random wrist poses 12–30 cm above the table, from straight above to oblique. `test/cube_monte_carlo.py` measures it (`--module` runs another version for comparison):

| 1200 random frames (paper cubes, table prior) | centre xy p50 / p95 / max | yaw p50 / p95 / max | ≤ 3 mm, ≤ 3° |
|---|---|---|---|
| before (one pose per face, weighted mean) | 1.6 / 7.6 / 28.6 mm | 1.3 / 5.2 / 40.2° | 89 %, 88 % |
| now, per frame | 1.7 / 3.1 / 6.1 mm | 1.4 / 3.5 / 8.9° | 94 %, 91 % |
| now, tracked over 6 wrist poses (100 cubes) | 1.5 / 2.5 / 3.5 mm | 1.4 / 2.8 / 3.6° | 98 %, 98 % |
| ideal cubes, per frame (300) | 0.6 / 1.1 / 1.6 mm | 0.2 / 0.8 / 1.6° | 100 %, 100 % |

The per-frame median is set by the printing errors (a marker 2 mm off-centre moves the cube by as much), which only seeing other faces averages out.

**Real frames** (`test/cube_real_frames.py ~/so101_data/cubes/20261122 --glob 'rest_*.jpg'`: 8 frames from the folded rest pose, camera 15 cm above the table, cubes 30–41 cm out, markers 19–33 px): five of the six cubes show two faces (top and a side), one only its top. The centres the two faces imply alone agree within 0.1–0.9 mm RMS (mean 0.5 mm), their yaws within 0.1–2° RMS (cube K1's two faces are 4° apart). Over the 8 frames a cube's estimate scatters by < 0.7 mm and < 0.2°. The paper cubes are thus better than the simulation assumes. The fit residual is larger, 0.7–1.3 px against 0.1–0.5 px for a single marker's own pose, and the free (unsnapped) face poses put the cubes ~4.5 mm below the table height: the upright cube on the table does not quite match the camera pose that the forward kinematics give. This points at the camera pose rather than the cubes, as the ruler check put edge and marker at nominal.

- The absolute accuracy also depends on the hand-eye transform and the joint origins, which were fitted before the shoulder zero changed (2026-11-14): refit them before trusting millimetres.
- For small or far cubes, run the camera at `scale:=1.0`: at 640×360 a marker 30 cm away is only ~27 px wide.
- Cost: ~2.5 ms for the markers and ~4 ms per cube for the fit on this machine, so ~25 ms for six cubes in view.

## Planned

- **Phase 5:** object detection beyond markers; outputs consumed by the Behavior Tree.
- Confidence / uncertainty outputs beyond the cubes (which have a confidence and spread, above), needed for the confidence-based handover trigger studied in phase 7.
