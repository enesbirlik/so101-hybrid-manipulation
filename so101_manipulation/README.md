# so101_manipulation

Cube pick and place for the SO-101: an action server that finds an ArUco cube with the wrist camera, grasps it from above without crushing it, and puts it down on the table or on another cube.

It grew out of the scratch scripts that grasped a toy 3/3 times on 2026-11-21 (`autograb.py`, `grasp_ik.py`, `vgrasp.py`) and the gripper measurements with a 30 mm paper cube on 2026-11-22.

## Run

```bash
cd ~/so101_ws && colcon build --packages-select so101_manipulation_interfaces so101_manipulation && source install/setup.bash

# with the control stack and the cube detector running (real arm: ask first, see CLAUDE.md)
ros2 launch so101_manipulation pick_place.launch.py
# or without ros2 launch / ros2 run:
install/so101_manipulation/lib/so101_manipulation/pick_place --ros-args --params-file install/so101_manipulation/share/so101_manipulation/config/pick_place.yaml

# the CLI (Ctrl-C cancels: the arm stops where it is, the gripper is left as it is)
install/so101_manipulation/lib/so101_manipulation/so101_cube pick 2 --hover-only   # dry run: stop at the hover
install/so101_manipulation/lib/so101_manipulation/so101_cube pick 2
install/so101_manipulation/lib/so101_manipulation/so101_cube place 25 -5           # cm, base_link
install/so101_manipulation/lib/so101_manipulation/so101_cube place --on 3          # stack on cube_3
install/so101_manipulation/lib/so101_manipulation/so101_cube open                  # open slowly where the arm is
```

Mock hardware (no camera, a FAKE cube, a simulated cube between the jaws):

```bash
export ROS_DOMAIN_ID=42
ros2 launch so101_moveit_config demo.launch.py rviz:=false &
ros2 launch so101_manipulation pick_place.launch.py fake_cube:=true fake_x:=0.22 fake_y:=0.03 fake_yaw_deg:=20 sim_contact_deg:=12.5
```

`sim_contact_deg` is refused unless `/robot_description` uses `mock_components/GenericSystem`.

## Interfaces

| Name | Type | |
|---|---|---|
| `/pick_cube` | `so101_manipulation_interfaces/action/PickCube` | goal `cube_id` ("2", "cube_2", "K2"), `hover_only`; result `success`, `message`, `failed_phase`, contact / hold / held angles, cube pose, wrist roll, jaw tilt, close-up shift; feedback `phase`, `detail`, `gripper_deg` |
| `/place_cube` | `so101_manipulation_interfaces/action/PlaceCube` | goal `x`, `y` [m] or `on_cube_id`, `yaw_deg` (NaN: any); result `success`, `message`, `failed_phase`, `placed_position` |
| `/cube_pick_place/open_gripper` | `std_srvs/Trigger` | open slowly where the arm is (the only "let go" besides PlaceCube) |
| `/cube_pick_place/overview` | `std_srvs/Trigger` | move to the overview pose, look at the table, answer with the cubes seen (JSON, below) |
| `/cube_pick_place/state` | `std_msgs/String`, transient local (latched), reliable, depth 1 | node state as JSON (below), on every change and at 1 Hz |
| `/cube_pick_place/reach` | `std_msgs/Float32MultiArray`, transient local (latched), depth 1 | reach table for the dashboard, one row per pan: `[pan_deg, near_m, far_m, near_all_m, far_all_m]` (below) |

**`/cube_pick_place/reach`**: published once at start-up from `config/reach.yaml` (`reach_file`), which `python3 -m so101_manipulation.reach_map --out DIR --yaml config/reach.yaml` computes offline with this planner (radius from base_link's origin, pans and cube yaws in base_link). `layout.dim[0]` = rows (label `pan`), `layout.dim[1]` = 5 columns (label lists them). `near_m` / `far_m`: the closest / farthest radius at which SOME tested cube yaw can be grasped (the band between them is contiguous at the 1 cm scan step); `near_all_m` / `far_all_m`: where EVERY tested yaw (0, 15, 30, 45 deg) can. NaN = nothing at that pan. The node warns when the table was computed with another `max_tilt_deg`, `max_close_elev_deg` or `roll_limit_deg`. Between the pans, interpolate; outside ±60 deg nothing was computed.

**`/cube_pick_place/overview`** (`std_srvs/Trigger`, empty request). Rejected at once (`success: false`, `{"error": "busy with a pick/place goal", "cubes": []}`) while a goal or service runs. Otherwise: jaws to `search_gripper_deg` (unless a cube is held: the grip stays and the cube counts in the clearance), a clearance-checked transit to each pose of `overview_poses_deg` (via a raised pose if needed), then a wait until `overview_min_frames` frames were processed after the arm stopped (+`obs_settle_s`) and every cube seen is stable, at most `overview_wait_s`. `success` = moved and saw frames from the detector. `message` is JSON:

```json
{"cubes": [{"id": "cube_2", "x_cm": 27.0, "y_cm": -10.0, "z_cm": 1.5, "yaw_deg": 9.8, "stable": true, "age_s": 0.06}],
 "frames": 8, "views": ["overview0"], "holding": null, "duration_s": 4.4}
```

`cubes`: every cube observed from the overview (base_link, cm; yaw folded to [−45°, 45°); `stable` as in `/cubes/tracked`), sorted by id; `holding`: the cube in the jaws or null; on failure an extra `"error": "..."` (and `success: false`). The arm stays at the overview pose afterwards, so a following pick of a cube seen there skips the search.

**`/cube_pick_place/state`** (`std_msgs/String`, transient local): JSON, sorted keys:

```json
{"state": "idle", "action": "", "cube": "", "phase": "", "held": "cube_2", "holding": "cube_2",
 "last": {"action": "pick", "cube": "cube_2", "success": true, "failed_phase": "",
          "timing_s": {"setup": 0.0, "search": 0.0, "approach": 2.8, "refine": 0.9, "descend": 3.1,
                       "grasp": 2.9, "verify": 0.0, "lift": 2.1, "total": 11.8}}}
```

- `state`: `"idle"` or `"busy"`; `action`: `"pick"`, `"place"`, `"overview"`, `"open_gripper"` or `""`; `cube`: the goal's cube; `phase`: the current phase.
- `held` (= `holding`): the cube the node believes is between the jaws (`"cube_<k>"`, or `"cube (held at start)"` for a place after a restart that found the jaws closed on something), or `null`. Set by a successful pick; cleared by a place, `open_gripper`, a cube lost in the lift, or a pick that finds the jaws empty. A restarted node starts with `null` (a PlaceCube then checks the grip itself).
- `last`: the last pick/place with its per-phase timing (below), or null.
| `/cubes/tracked`, `/cubes/detections` | `vision_msgs/Detection3DArray` | in: cube poses from `so101_perception` (see `cube_source`) |
| `/arm_controller/…`, `/gripper_controller/…` | `FollowJointTrajectory`, `JointTrajectoryControllerState` | out / in: motion and lag |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | in: gripper load, current, `torque_limit` |
| `/gripper_torque_controller/commands` | `std_msgs/Float64MultiArray` | out: gripper torque limit during the grasp |

Custom actions because no standard type fits a "pick cube k" goal with these phases and results; they live in the small `so101_manipulation_interfaces` package (rosidl needs ament_cmake) so that clients (CLI, a future Behavior Tree in `so101_bt`) depend only on it.

## Phases

**PickCube**

1. **search** (details below): none at all when the cube is fresh and stable; else a short look where the arm is, then the views in order: aimed at where the cube was last seen, the overview, autograb's views.
2. **approach**: plan the grasp (below), transit to the hover pose, opening the jaws to `open_deg` on the way (`open_during_approach`: the path is checked for the closed, half-open and open jaws; the jaws are opened only where the open jaws clear the table by 2 cm).
3. **refine**: at the hover, wait for a fresh stable estimate (observations only count from `obs_settle_s` after the arm stopped), re-target when it moved > `refine_min_shift_mm`, at most `refine_max_iter` times; abort beyond `refine_max_shift_mm`. Any face of the cube counts (the 2026-11-21 script skipped the refinement when the close-up saw a different part of the object); the yaw is compared modulo 90°.
4. **descend**: torque limit lowered (and confirmed) at the hover; the hover trim takes the servos' static error out and is reused below (short passes: measure after `trim_settle_s`, apply the whole error in one `trim_move_s` move, a second pass only if still beyond `trim_tol_deg`); then one continuous move down to the grasp height through the `descend_step` points; contact stop when the `contact_joint` lag (reference − feedback) rises more than `descend_contact_lag_deg` over the hover baseline: back up to the hover and abort.
5. **grasp**: the soft grasp (below).
6. **verify**: held / empty / uncertain from the jaw angle and current. Not held: open slowly, rise to the hover, abort.
7. **lift**: back to the hover in one continuous move through the descent points (with the hover trim's bias, no stop at each point); for a tilted grasp that is back out ALONG THE JAW AXIS first, then straight up to the exit pose (`GraspPlan.retreat_poses`); judge the grip again (lost: abort, gripper left as it is). A miss at the bottom (back off) and a contact stop on the way down retreat along the same line.

**PlaceCube**: **carry** (transit to the hover over the target, the carried cube counted in the clearance), **refine** (stacking only: close-up of the lower cube), **descend** (contact stop = the cube is down), **release** (open slowly, then restore the torque limit), **retreat** (back up to the hover, one continuous move with the trim bias). Stacking aligns the closing direction with the lower cube's faces and puts the release at its top.

### Search

The node keeps, for the whole session, where every cube was last seen (newest `/cubes/tracked` or `/cubes/detections` observation; a picked cube is "in the jaws", a placed one is where it was put). A search:

1. **Skip**: a stable estimate with `obs_min_fresh` observations from the last `search_fresh_s` (e.g. right after an overview, or when the cube is in view) is used at once: no wait, no move. (Before, the search always waited for observations newer than the goal, ~0.3–1 s.)
2. **Look where the arm is** for up to `search_initial_wait_s`.
3. **Views**, in order; at each, observations count from `obs_settle_s` after the arm stopped, for up to `search_view_wait_s`, but the view is left as soon as `search_empty_frames` processed frames (the detector publishes `/cubes/detections` for every frame taken while the arm stands still, also an empty one) came without the cube:
   - **aimed**: the camera looking down at the cube's last known position (`grasp.aim_view_pose`, from the first of `search_aim_heights` whose image shows the top face past the jaws in the fisheye model);
   - the **overview** pose(s);
   - autograb's views (*near* camera ~20 cm above the table, *close* 17 cm out, *far* 33 cm out, each at `search_pans_deg`), sorted by how close their optical axis points to the last known position.

The jaws go to `search_gripper_deg` (never below the floor) while searching.

### Overview pose

`overview_poses_deg` = **[22.8, −77.0, 40.9, 89.4, −72.4]°** (pan, lift, elbow, wrist_flex, roll), from `grasp.overview_pose()`: start with the camera ~28 cm above the table looking straight down at 22 cm out (least squares like `view_pose`, pan and roll free: the camera sits beside the jaws, so looking straight down takes a large wrist_roll), then a seeded random local search on the coverage itself, keeping |roll| ≤ 85°, wrist_flex 5° inside its limit, the camera ≥ 24 cm up, the arm ≥ 6 cm above the table. Coverage (`grasp.view_coverage`) = the share of cube positions on a 1 cm grid whose top face is in the 1280×720 fisheye image (equidistant model of `wrist_camera_fisheye.yaml`, 10 px margin, < 85° off axis), not behind the jaws (projected hulls of `gripper_link` and the moving jaw at `search_gripper_deg`), the 22 mm top marker ≥ 30 px (15 px at the detector's 640 px; the smallest is 42 px) and seen at a view cosine ≥ 0.3.

| | camera above the table | 15–30 cm × ±15 cm | 15–30 × ±20 cm | 12–34 × ±22 cm |
|---|---|---|---|---|
| overview | 24.8 cm, axis 15° off vertical | **99 %** (misses 5 points at 15–16 cm, y −1…+3 cm: behind the jaws) | 88 % | 72 % |
| autograb's *near* view (pan 0) | 19.9 cm | 48 % | | |

One pose is enough, so there is one. Recompute after a new hand-eye or kinematic calibration (~40 s):

```bash
python3 -c "import math, numpy as np; from so101_manipulation import grasp as g; from so101_manipulation.kinematics import ArmModel, perception_config_dir as d; m = ArmModel(); q, f = g.overview_pose(m, g.load_hand_eye(d() / 'wrist_camera_hand_eye.yaml'), g.load_fisheye(d() / 'wrist_camera_fisheye.yaml')); print(np.degrees(q).round(1).tolist(), f)"
```

The node logs the coverage of the configured pose(s) at startup.

### Timing

Every pick and place times its phases (`timing.PhaseTimer`; a retried phase adds up). The result `message` ends with `| timing [s]: setup 0.0, search 0.0, approach 2.8, refine 0.9, descend 3.1, grasp 2.9, verify 0.0, lift 2.1, total 11.8` (also on failure and cancel), the node logs the same line, and `~/state` carries it as `last.timing_s`. `descend` includes the torque-limit check and the hover trim; `grasp` the close and the grip settle. Place: `setup, carry, refine (stacking), descend, release, retreat, total`.

## Grasp geometry (`grasp.py`)

- `gripper_frame_link` is the **fixed** jaw tip. At 30° open the gap midpoint is ~3 cm beside it, so the IK puts the point `gap_fraction` of the way from the fixed to the moving jaw tip (moving jaw tip = the mesh vertex farthest from its joint) over the cube centre, and the fixed tip at the grasp height.
- The jaws close along a face normal: 4 options (`yaw + k·90°`). Options whose wrist roll would be beyond `roll_limit_deg` are dropped (the camera cable tore at −124°); the rest are solved (hover, every descent step, grasp, with the same closing direction) and the one with a good IK and the smallest |roll| wins.
- Vertical jaws reach ~14-30 cm out. Elsewhere (`allow_tilt`) the **tilt sweep** (below), else a clear "reach" message. The hover is lowered from `hover_height` towards `hover_min_height` when needed. A cube on another cube is grasped relative to its own bottom.
- **Tilt sweep** (`plan_tilted`, replaces the weighted "tilted" pass that preferred missing the cube by 2-4 cm over leaning more than ~3.5°). The 5-DOF arm keeps the jaw axis in its own vertical plane, so the jaws can only LEAN in that plane: tips away from the base for far cubes (the approach comes from the robot's side), towards the base for near ones. Leans `±tilt_step_deg, ±2·step, …` up to `max_tilt_deg`, smallest first; for each lean and closing option one exact solve of 5 equations in 5 joints: grasp point (3, hard, `ik_tolerance_mm`), lean (1, hard), closing direction along the face normal (1, soft within `close_tol` 3°, because |wrist_roll| ≤ 90° can leave it 1-2° short). The first lean with a full plan wins; among its options the smallest |roll| and the most horizontal closing direction. `max_close_elev_deg` bounds how far the closing direction may tip out of the horizontal (a jaw closing along the leaning plane meets the cube's vertical faces at that angle).
- Grasp point of a tilted grasp: the gap point lies on the jaw axis through the cube centre at **mid-height**, `d` beyond it, with `d` such that vertical jaws would put it where the vertical planner does: the pads stay centred on the cube's mid-height as the jaws lean. If the lowest point of the hand (table model, moving jaw included) is below `min_final_clearance_mm`, the grasp point rises in 2 mm steps (≤ 6 mm). The gap-midpoint correction is the same 3-D point in `gripper_link`, so it is exact in the tilted frame.
- Approach of a tilted grasp: the hover is `hover_height − grasp_tip_height` back **along the jaw axis** from the grasp point, and the descent follows that line (the jaws slide onto the cube); if that line runs out of the workspace (near the base wrist_flex is at its limit) the descent goes straight down with the jaws leaning. Lift: back along the line to the hover, then up to an exit pose (the height the leaning line gave away + 2 cm, if reachable).
- Contact on the way down: `contact_watch` takes the joint a push against the descent line deflects most (`J_k · line` at the grasp pose; the lag of joint k is proportional to it). That is shoulder_lift with a positive lag for every grasp checked, vertical or leaning to 45°, so the existing threshold applies; `tilt_contact_joint` overrides it for tilted grasps.
- A **tilted hold is placed with the same lean and wrist_roll** (`plan_place_held`: the only hand orientations that keep the cube flat are the grasp's turned about the vertical), straight down onto the target or, far out, along the jaw axis. The arm reaches that orientation only in a band of radii, so if the cube cannot go down flat the lean moves towards vertical in 5° steps, at most `PLACE_MAX_TILT_DEG` (10°): the cube then lands on one bottom edge (released with that edge `release_gap` above the surface) and settles flat when let go; the plan summary says so. Measured offline: a cube picked at 40 cm (30° lean) can be put down at 18-30 cm (flat at 22 and 30 cm, 5° elsewhere); one picked at 12 cm (10° inward lean) only at ~20-25 cm. Yaw and stacking alignment are not chosen for a tilted hold (they follow from the pan). Outside the band the place fails in `carry` with the reason.

**Reach** (`config/reach.yaml`, 2026-11-22: 30 mm cube, 30° open, max lean 45°, step 5°, closing ≤ 20° off horizontal, |roll| ≤ 90°; cube yaws 0/15/30/45° in base_link; radius from base_link's origin, 1 cm steps). Before the tilt sweep: 14-30 cm at pan 0, 12-30 cm at ±60°.

| pan | near (some yaw) | far (some yaw) | near (all yaws) | far (all yaws) |
|---|---|---|---|---|
| −60 | 8 | 41 | 11 | 39 |
| −45 | 9 | 42 | 12 | 37 |
| −30 | 10 | 43 | 13 | 38 |
| −15 | 10 | 43 | 12 | 38 |
| 0 | 11 | 43 | 14 | 38 |
| +15 | 11 | 43 | 13 | 40 |
| +30 | 10 | 43 | 13 | 39 |
| +45 | 9 | 42 | 11 | 37 |
| +60 | 9 | 41 | 11 | 36 |

At the far end the jaws lean the full 45°. What stops it: for a cube whose face normal is within ~20° of the radial direction, the arm's length (at 44 cm the IK misses by 3-5 mm; a 60° lean would add ~1 cm, to 44 cm, nothing reaches 46+); for other yaws, the closing direction: the jaws can only lean in the arm's plane, so a cube turned 25-45° to the radial direction is gripped with the closing direction tipped out of the horizontal by asin(sin(yaw)·sin(lean)) > 20°, and the two other closing options need wrist_roll ±135° (camera cable). At pan 0: any yaw up to 37-38 cm, at 40 cm yaws within ±25° of radial, at 42 cm within ±20°. At the near end shoulder_lift is at its limit (−97°) and wrist_flex at 95°, with an inward lean of 10-20°.

- The vertical residual leaves ~2° of tilt: the wrist_roll joint axis is ~2.8° off the jaw direction in the URDF, and a 5-DOF arm cannot cancel that while also fixing the closing direction.

## Soft grasp (`soft_grasp.py`)

A 30 mm paper cube was crushed by a position-controlled close far below its width. The rules now:

1. **Hard command floor.** No gripper command ever goes below the angle of a jaw gap of `cube_edge_mm − squeeze_mm` (gap ≈ `gap_zero_mm` + `gap_per_deg_mm` × angle; 30 mm, 1.5 mm → **11.48°**). The closing ramp ends there, hold targets are clamped to it, and `motion.py` clamps every gripper goal the node sends. The servo is position-controlled, so this alone bounds the force. Measured: an empty gripper commanded to 11.48° stops at 12.04° (deadband), the paper cube at 12.48–12.57° held steadily for 10 s, undamaged.
2. **Torque limit** `gripper_torque_limit` (200 = 20 %, enough for a paper cube) during the grasp: published repeatedly until the `torque_limit` readback in `/dynamic_joint_states` matches (a single publish was lost on the real arm), abort if not within `torque_limit_timeout_s`; restored to `gripper_torque_restore` only after the gripper has opened. Skipped on mock hardware; on the real arm a missing interface aborts unless `torque_limit_required` is false.
3. **Two-speed close**, with contact detection on the way: fast (`close_fast_speed_deg_s`) from `open_deg` down to `expected + contact_window_deg + close_slow_lead_deg` (21.1° for 30 mm), where no contact can count anyway, then slow (`close_speed_deg_s`) through the whole window down to the floor (`soft_grasp.close_ramp`; 2.0 s instead of 3.1 s). The slow part is sent in 1° pieces with explicit velocities (`ramp_trajectory`) so the controller's cubic segments are straight lines there and the last piece ends at rest exactly at the floor without overshoot (positions-only inner points made the mock controller dip to 11.14°, below the floor). Only samples of the slow part train the contact baseline (`ContactDetector.baseline_below_deg`): the larger tracking lag of the fast part would make a contact show late. Sign: the controller lag (commanded − measured) goes *negative* when the jaws are blocked (free closing −0.3…−0.5°). The detector uses `measured − commanded` above its running free-closing baseline > `contact_lag_deg` for `contact_samples` samples (optionally load). A contact above `expected + contact_window_deg` (expected 12.6° for 30 mm) means something wider is between the jaws: freeze, open, back off, retry (`grasp_retries`) or abort. A contact inside the window moves the command to max(floor, contact − `squeeze_deg`). Note: with the floor this close to the cube the lag barely moves (−0.7° on the real arm), so the held decision does not rely on it.
4. **Held or not**, standing at the final command `c` after `grip_settle_s`: **held** if the jaws stopped above `c + empty_deadband_deg + held_angle_margin_deg` (12.29° for 30 mm) or the current averaged over `current_window_s` is ≥ `held_current_ma`; **empty** if at or below `c + empty_deadband_deg + empty_angle_tol_deg` with current < `empty_current_ma`; anything else is **uncertain** and treated as not held. The same check runs again after the lift.

Mock hardware has no contact and no deadband: without `sim_contact_deg` a pick ends cleanly "not held" (jaws at the floor = empty); with it, the measured angle is replaced by max(actual, `sim_contact_deg`).

## Motion and safety

- **Direct joint trajectories, not MoveIt, for every move.** MoveIt knows neither the table height nor the cubes, Pilz cannot plan the gripper, and the grasp needs constraints pick_ik does not express (gap midpoint, closing direction, roll limit). Straight joint-space moves timed from the largest joint change were reliable on this arm (2026-11-21), and direct goals make the contact stops (cancel → the controller holds) and the gripper preemption simple.
- **Clearance**: every transit segment is sampled against the table model (convex hulls of every link mesh and the wrist camera, `table_z` from `so101_kinematics.yaml`) plus a carried cube; the lowest point must stay ≥ `transit_clearance_mm` (cubes are 30 mm tall). Leaving or reaching a low pose only requires not going lower. Candidates: direct, via a pose `via_height` above the target, via a raised current pose; none clear → abort without moving.
- **Limits**: every arm goal is checked against the planning limits and |wrist_roll| ≤ `roll_limit_deg` before it is sent; IK bounds keep the roll inside as well.
- **Speeds** are parameters (`transit_speed`, `approach_speed`, `descend_speed`, `lift_speed`, gripper `close_speed_deg_s` / `open_speed_deg_s`).
- **Cancel** (Ctrl-C in the CLI, or cancel the goal): active controller goals are canceled, so the joints hold where they are; the gripper is never opened by a cancel. One goal at a time; others are rejected while busy.
- The gripper opens only in: approach (during the move to the hover, from a pose where the open jaws clear the table by 2 cm, along a path checked for every jaw angle), a miss at the bottom (after "not held"), release, and the `open_gripper` service.

## Perception adapter (`cube_tracker.py`)

`CubeTracker.get(cube_id, since)` → `CubeEstimate(centre, yaw, stable, spread_mm, yaw_spread_deg, age_s, count)` is the only place the node reads perception. `cube_source`:

- `tracked`: `/cubes/tracked`, the detector's temporal filter (score ≥ 0.5 = stable, spread from the covariance, stamp = last image the cube was seen in). Stable for the node also needs `obs_min_fresh` new stamps after `since`; if the filter still lags (old far-view observations in its window), `obs_min_count` fresh observations that agree count as stable with their own median.
- `detections`: `/cubes/detections`, the node's own median window (`obs_*`).
- `auto` (default): tracked for a cube the filter reported in the last 2 s, else detections.

## Tests

```bash
cd so101_manipulation && python3 -m pytest -q test          # no ROS graph needed: geometry, soft grasp, close ramp, search views, overview coverage, adapter, timing, torque hook, params
export ROS_DOMAIN_ID=42                                       # mock integration test, see its docstring
ros2 launch so101_moveit_config demo.launch.py rviz:=false &
python3 test/smoke_mock.py                                    # miss, pick_place, search, reach, cancel, overview, memory, far, far40, near, near11
```

`test/smoke_mock.py` starts its own FAKE cube publisher (`fake_cube_publisher`: `/cubes/detections` and `/cubes/tracked`, frames ignored (not published) while the arm moves, optional field-of-view cone, `fov_check` through the fisheye model, far-view bias) and node per scenario, and refuses to run on domain 0 or on real hardware.

## Files

| File | Content |
|---|---|
| `so101_manipulation/pick_place_node.py` | action server, phases, parameters (`PARAMS`) |
| `so101_manipulation/grasp.py` | grasp IK, closing options, reach, view poses, fisheye visibility / coverage, overview and aimed views, search order |
| `so101_manipulation/soft_grasp.py` | floor, contact detector, two-speed close ramp, hold target, held / empty |
| `so101_manipulation/timing.py` | per-phase timer |
| `so101_manipulation/reach_map.py` | offline reach map with the planner (`python3 -m so101_manipulation.reach_map`), the `~/reach` publisher |
| `config/reach.yaml` | the reach table `~/reach` publishes |
| `so101_manipulation/kinematics.py` | FK from the URDF + `so101_kinematics.yaml`, mesh hulls, table clearance (no ROS) |
| `so101_manipulation/motion.py` | trajectories to the controllers, limit and floor checks, lag / current / torque-limit state |
| `so101_manipulation/cube_tracker.py` | perception adapter |
| `so101_manipulation/hooks.py` | gripper torque limit (publish until the readback matches) |
| `so101_manipulation/cli.py`, `fake_cube_publisher.py` | `so101_cube` CLI, mock cube |
| `config/pick_place.yaml` | every parameter with its default (kept equal to `PARAMS` by `test/test_params.py`) |

## Open points (real arm)

- **Tilted grasps (2026-11-22) have run on mock only.** Unknown on the real arm: the servo sag with the arm stretched to 40+ cm (the hover trim corrects it at the hover, the descent reuses the bias), whether the shoulder_lift lag still shows a jaw landing on a paper cube when the descent is mostly horizontal, how a paper cube behaves when the jaw pads meet it at up to `max_close_elev_deg` (20°) out of the horizontal, the camera's close-up view from a leaning hover, and placing a tilted hold. Start with `hover_only` picks at 34-38 cm.

- `grasp_tip_height` 11 mm with `table_z` known to ~2.4 mm rms: the moving jaw hangs ~6 mm lower than the fixed tip at 30° open (lowest point ~4.7 mm above the table in the model).
- The descent contact thresholds come from the toy grasp; a paper cube may give a weaker shoulder_lift lag when a jaw lands on it.
- Close-up visibility of the cube at the hover with the jaws open, and the far/close accuracy, are from one session.
- Stacking (`place --on`) has not run on mock with two cubes or on the real arm.
- Overview pose, aimed views, early view exit, two-speed close, short hover trim and continuous lift/retreat (2026-11-22) have run on mock only. The coverage is a model: the jaw silhouette is the convex hull of the meshes, the marker size and view-angle limits are estimates, and the detector's real range at the image border is untested. The overview needs wrist_roll −72° and wrist_flex 89°.
- `search_empty_frames` 6 (~0.4 s at 15 fps) may leave a view where a small, far cube is only found in some frames; raise it if searches miss cubes that are in view.
- Hover trim with one 0.4 s move and 0.25 s settles: the measurement after 0.25 s may still be settling (stick-slip at P 16); `trim_max_iter` 2 allows one more pass.

## Parameters

All in `config/pick_place.yaml`. Lengths in m unless `_mm`, angles in degrees, speeds in rad/s on the joint that moves furthest unless `_deg_s`.

**Perception**

| Parameter | Default | Meaning |
|---|---|---|
| `cube_source` | `auto` | cube adapter: 'tracked' (/cubes/tracked, the detector's filter), 'detections' (/cubes/detections + own median window), 'auto' (tracked when it reports the cube) |
| `tracked_topic` | `/cubes/tracked` | filtered cubes (vision_msgs/Detection3DArray, score >= 0.5 = stable) |
| `detections_topic` | `/cubes/detections` | per-frame cubes (vision_msgs/Detection3DArray) |
| `obs_min_fresh` | `3` | tracked: new image stamps after the arm stopped needed for a fresh stable estimate |
| `cube_edge_mm` | `30.0` | cube edge |
| `obs_window_s` | `1.5` | observations older than this (s) are not used for an estimate |
| `obs_min_count` | `5` | observations needed for a stable estimate |
| `obs_max_spread_mm` | `3.0` | stable: every observation within this of the median centre |
| `obs_max_yaw_spread_deg` | `4.0` | stable: every yaw (mod 90) within this of the median |
| `obs_settle_s` | `0.4` | after the arm stops, wait this long before observations count |

**Search**

| Parameter | Default | Meaning |
|---|---|---|
| `search_pans_deg` | `[0.0, -25.0, 25.0, -45.0, 45.0]` | pan angles of the search views |
| `search_initial_wait_s` | `1.0` | first look for the cube where the arm is, this long |
| `search_view_wait_s` | `2.0` | wait for a stable cube at each view pose |
| `search_gripper_deg` | `12.0` | jaws nearly shut while searching (they hide less of the image); clamped to the floor |
| `search_fresh_s` | `1.0` | a stable estimate with obs_min_fresh observations from the last this many s skips the search |
| `search_empty_frames` | `6` | leave a view early once this many processed frames (/cubes/detections) after the settle did not show the cube; 0 = always wait search_view_wait_s |
| `search_aim_heights` | `[0.22, 0.26, 0.18]` | camera heights [m] tried for the first view, aimed at where the cube was last seen (the first that sees it wins) |

**Overview**

| Parameter | Default | Meaning |
|---|---|---|
| `overview_poses_deg` | `[22.8, -77.0, 40.9, 89.4, -72.4]` | overview pose(s), 5 joints each (pan, lift, elbow, wrist_flex, roll) [deg], flat list; grasp.overview_pose() |
| `overview_wait_s` | `3.0` | overview: wait at most this long per pose for fresh frames and stable tracks |
| `overview_min_frames` | `8` | overview: processed frames to see after the arm stopped |

**Grasp geometry**

| Parameter | Default | Meaning |
|---|---|---|
| `open_deg` | `30.0` | gripper opening for the approach (~53 mm gap) |
| `grasp_tip_height` | `0.011` | fixed jaw tip above the cube's bottom at the grasp |
| `hover_height` | `0.07` | fixed jaw tip above the cube's bottom at the hover |
| `hover_min_height` | `0.05` | lowest hover tried when the hover is out of reach |
| `descend_step` | `0.01` | height step of the descent |
| `gap_fraction` | `0.5` | where the cube centre goes between the fixed (0) and moving (1) jaw tip; 0.5 = gap midpoint |
| `allow_tilt` | `true` | allow a tilted approach (<= max_tilt_deg) where vertical jaws do not reach |
| `max_tilt_deg` | `45.0` | largest jaw lean of the tilt sweep (far cubes lean out, near cubes lean in) |
| `tilt_step_deg` | `5.0` | lean step of the tilt sweep; the smallest lean that works is used |
| `max_close_elev_deg` | `20.0` | tilt sweep: closing direction at most this far off horizontal (jaw pads vs the cube's vertical faces) |
| `tilt_contact_joint` | `auto` | contact joint for TILTED grasps: 'auto' = the joint a push against the jaw axis deflects most (Jacobian), or a joint name; vertical grasps use contact_joint |
| `reach_file` | `""` | reach table published on ~/reach (python3 -m so101_manipulation.reach_map); '' = config/reach.yaml of this package |
| `vertical_tol_deg` | `3.0` | jaws count as vertical within this |
| `ik_tolerance_mm` | `2.0` | largest IK position error accepted |
| `roll_limit_deg` | `90.0` | /wrist_roll/ never beyond this (camera cable) |
| `min_final_clearance_mm` | `3.0` | lowest point of the hand above the table at the grasp pose |

**Close-up refinement**

| Parameter | Default | Meaning |
|---|---|---|
| `refine_max_iter` | `2` | close-up refinements at the hover |
| `trim_tol_deg` | `0.3` | hover trim: stop when every measured arm joint is within this of the plan |
| `trim_max_iter` | `2` | hover trim: correction moves at most (each applies the whole measured error) |
| `trim_max_bias_deg` | `5.0` | hover trim: largest correction added to a joint command |
| `trim_move_s` | `0.4` | hover trim: duration of one correction move |
| `trim_settle_s` | `0.25` | hover trim: wait before each measurement |
| `refine_min_shift_mm` | `2.0` | re-target when the close-up estimate moved more than this |
| `refine_max_shift_mm` | `40.0` | abort when the close-up estimate moved more than this |
| `refine_timeout_s` | `3.0` | wait this long for a stable close-up estimate |
| `refine_required` | `false` | abort when the close-up sees no stable cube (else keep the far estimate) |

**Contact on the way down (arm)**

| Parameter | Default | Meaning |
|---|---|---|
| `contact_joint` | `shoulder_lift` | joint whose lag (reference - feedback) shows a contact |
| `descend_contact_lag_deg` | `2.5` | lag above the hover baseline during the descent = contact |
| `descend_lag_samples` | `15` | controller states averaged after each step |

**Soft grasp (gripper)**

| Parameter | Default | Meaning |
|---|---|---|
| `close_speed_deg_s` | `6.0` | closing ramp speed inside the contact window (and close_slow_lead_deg above it) |
| `close_fast_speed_deg_s` | `20.0` | closing speed above the contact window + close_slow_lead_deg (no contact can count there); <= close_speed_deg_s: one slow ramp from open |
| `close_slow_lead_deg` | `3.5` | the slow part starts this far above the window (the contact baseline warms up) |
| `open_during_approach` | `true` | open the jaws during the move to the hover (the path is checked for closed, half-open and open jaws) instead of before it |
| `open_speed_deg_s` | `20.0` | opening speed (release, abort) |
| `contact_lag_deg` | `1.0` | contact when measured - commanded (= -(controller lag), positive when blocked) is this far above its free-closing baseline |
| `contact_samples` | `3` | consecutive samples over the threshold |
| `contact_load_pct` | `0.0` | /load - baseline/ that also counts as contact; 0 = off |
| `squeeze_deg` | `1.5` | hold target = contact angle - this, never below the floor |
| `squeeze_mm` | `1.5` | HARD FLOOR: no gripper command below the angle of a gap of cube_edge_mm - this (30 mm -> ~11.5 deg); the closing ramp ends there |
| `contact_window_deg` | `5.0` | contact must lie within expected +- this, else abort |
| `gap_zero_mm` | `13.0` | jaw gap at 0 deg |
| `gap_per_deg_mm` | `1.35` | jaw gap per degree (<= 30 deg) |
| `empty_deadband_deg` | `0.56` | empty jaws stop this far above the command (servo deadband; real arm: command 11.48 -> 12.04 deg) |
| `held_angle_margin_deg` | `0.25` | held: measured > command + empty_deadband_deg + this |
| `empty_angle_tol_deg` | `0.15` | empty: measured <= command + empty_deadband_deg + this (and low current) |
| `held_current_ma` | `12.0` | held: or the gripper current at/above this (real arm: 13-20 mA with the cube, 6 empty) |
| `empty_current_ma` | `10.0` | empty: current below this |
| `current_window_s` | `0.3` | the current is averaged over this |
| `grip_settle_s` | `0.4` | wait after the close before judging the grip |
| `hold_ramp_s` | `0.15` | time to move the command to the hold target |
| `grasp_retries` | `1` | retries after a contact outside the window |
| `sim_contact_deg` | `NaN` | MOCK ONLY: pretend a cube blocks the jaws at this angle (NaN = off); refused unless robot_description uses mock_components |
| `gripper_torque_limit` | `200.0` | gripper torque limit during the grasp, 0.1 % units (200 = 20 %, holds a paper cube); <= 0 = do not touch it |
| `gripper_torque_restore` | `1000.0` | torque limit restored after the gripper opened (clamped by the hardware) |
| `torque_limit_topic` | `/gripper_torque_controller/commands` | std_msgs/Float64MultiArray command topic |
| `torque_limit_timeout_s` | `3.0` | abort when the torque_limit readback does not match within this |
| `torque_limit_required` | `true` | abort when the torque limit interface is missing (skipped on mock hardware) |

**Place**

| Parameter | Default | Meaning |
|---|---|---|
| `release_gap` | `0.002` | release this much above the surface (contact stops earlier) |
| `stack_max_age_s` | `60.0` | the cube to stack on must have been seen this recently |

**Motion / safety**

| Parameter | Default | Meaning |
|---|---|---|
| `transit_speed` | `0.9` | rad/s, transit |
| `approach_speed` | `0.45` | rad/s, re-targeting at the hover |
| `descend_speed` | `0.2` | rad/s, descent steps |
| `lift_speed` | `0.3` | rad/s, lift / retreat |
| `min_move_s` | `0.8` | shortest move |
| `settle_s` | `0.3` | pause after each move |
| `transit_clearance_mm` | `45.0` | lowest point (arm, camera, carried cube) above the table on transits; cubes are 30 mm tall |
| `via_height` | `0.12` | tip height of the via pose above a target |
| `table_z` | `NaN` | table surface in base_link; NaN = so101_kinematics.yaml |
