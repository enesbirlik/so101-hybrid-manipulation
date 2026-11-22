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
| `/cubes/tracked`, `/cubes/detections` | `vision_msgs/Detection3DArray` | in: cube poses from `so101_perception` (see `cube_source`) |
| `/arm_controller/…`, `/gripper_controller/…` | `FollowJointTrajectory`, `JointTrajectoryControllerState` | out / in: motion and lag |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | in: gripper load, current, `torque_limit` |
| `/gripper_torque_controller/commands` | `std_msgs/Float64MultiArray` | out: gripper torque limit during the grasp |

Custom actions because no standard type fits a "pick cube k" goal with these phases and results; they live in the small `so101_manipulation_interfaces` package (rosidl needs ament_cmake) so that clients (CLI, a future Behavior Tree in `so101_bt`) depend only on it.

## Phases

**PickCube**

1. **search**: wait `search_initial_wait_s` for a stable estimate where the arm is; else sweep the view poses (autograb's: *near* camera ~20 cm above the table, *close* 17 cm out, *far* 33 cm out, each at `search_pans_deg`), sorted by how close the camera axis points to where the cube was last seen. The jaws go to `search_gripper_deg` (never below the floor).
2. **approach**: plan the grasp (below), open to `open_deg`, transit to the hover pose.
3. **refine**: at the hover, wait for a fresh stable estimate (observations only count from `obs_settle_s` after the arm stopped), re-target when it moved > `refine_min_shift_mm`, at most `refine_max_iter` times; abort beyond `refine_max_shift_mm`. Any face of the cube counts (the 2026-11-21 script skipped the refinement when the close-up saw a different part of the object); the yaw is compared modulo 90°.
4. **descend**: torque limit lowered (and confirmed) at the hover, then down in `descend_step` steps; contact stop on `contact_joint` lag (reference − feedback, over the hover baseline) > `descend_contact_lag_deg` after a step or > `motion_contact_lag_deg` during one: back up one step and abort.
5. **grasp**: the soft grasp (below).
6. **verify**: held / empty / uncertain from the jaw angle and current. Not held: open slowly, rise to the hover, abort.
7. **lift**: straight back up to the hover, judge the grip again (lost: abort, gripper left as it is).

**PlaceCube**: **carry** (transit to the hover over the target, the carried cube counted in the clearance), **refine** (stacking only: close-up of the lower cube), **descend** (contact stop = the cube is down), **release** (open slowly, then restore the torque limit), **retreat** (back up to the hover). Stacking aligns the closing direction with the lower cube's faces and puts the release at its top.

## Grasp geometry (`grasp.py`)

- `gripper_frame_link` is the **fixed** jaw tip. At 30° open the gap midpoint is ~3 cm beside it, so the IK puts the point `gap_fraction` of the way from the fixed to the moving jaw tip (moving jaw tip = the mesh vertex farthest from its joint) over the cube centre, and the fixed tip at the grasp height.
- The jaws close along a face normal: 4 options (`yaw + k·90°`). Options whose wrist roll would be beyond `roll_limit_deg` are dropped (the camera cable tore at −124°); the rest are solved (hover, every descent step, grasp, with the same closing direction) and the one with a good IK and the smallest |roll| wins.
- Vertical jaws reach ~30 cm out. Beyond, a tilted approach up to `max_tilt_deg` (`allow_tilt`), else a clear "reach" message. The hover is lowered from `hover_height` towards `hover_min_height` when needed. A cube on another cube is grasped relative to its own bottom.
- The vertical residual leaves ~2° of tilt: the wrist_roll joint axis is ~2.8° off the jaw direction in the URDF, and a 5-DOF arm cannot cancel that while also fixing the closing direction.

## Soft grasp (`soft_grasp.py`)

A 30 mm paper cube was crushed by a position-controlled close far below its width. The rules now:

1. **Hard command floor.** No gripper command ever goes below the angle of a jaw gap of `cube_edge_mm − squeeze_mm` (gap ≈ `gap_zero_mm` + `gap_per_deg_mm` × angle; 30 mm, 1.5 mm → **11.48°**). The closing ramp ends there, hold targets are clamped to it, and `motion.py` clamps every gripper goal the node sends. The servo is position-controlled, so this alone bounds the force. Measured: an empty gripper commanded to 11.48° stops at 12.04° (deadband), the paper cube at 12.48–12.57° held steadily for 10 s, undamaged.
2. **Torque limit** `gripper_torque_limit` (200 = 20 %, enough for a paper cube) during the grasp: published repeatedly until the `torque_limit` readback in `/dynamic_joint_states` matches (a single publish was lost on the real arm), abort if not within `torque_limit_timeout_s`; restored to `gripper_torque_restore` only after the gripper has opened. Skipped on mock hardware; on the real arm a missing interface aborts unless `torque_limit_required` is false.
3. **Slow close** at `close_speed_deg_s` from `open_deg` to the floor, with contact detection on the way. Sign: the controller lag (commanded − measured) goes *negative* when the jaws are blocked (free closing −0.3…−0.5°). The detector uses `measured − commanded` above its running free-closing baseline > `contact_lag_deg` for `contact_samples` samples (optionally load). A contact above `expected + contact_window_deg` (expected 12.6° for 30 mm) means something wider is between the jaws: freeze, open, back off, retry (`grasp_retries`) or abort. A contact inside the window moves the command to max(floor, contact − `squeeze_deg`). Note: with the floor this close to the cube the lag barely moves (−0.7° on the real arm), so the held decision does not rely on it.
4. **Held or not**, standing at the final command `c` after `grip_settle_s`: **held** if the jaws stopped above `c + empty_deadband_deg + held_angle_margin_deg` (12.29° for 30 mm) or the current averaged over `current_window_s` is ≥ `held_current_ma`; **empty** if at or below `c + empty_deadband_deg + empty_angle_tol_deg` with current < `empty_current_ma`; anything else is **uncertain** and treated as not held. The same check runs again after the lift.

Mock hardware has no contact and no deadband: without `sim_contact_deg` a pick ends cleanly "not held" (jaws at the floor = empty); with it, the measured angle is replaced by max(actual, `sim_contact_deg`).

## Motion and safety

- **Direct joint trajectories, not MoveIt, for every move.** MoveIt knows neither the table height nor the cubes, Pilz cannot plan the gripper, and the grasp needs constraints pick_ik does not express (gap midpoint, closing direction, roll limit). Straight joint-space moves timed from the largest joint change were reliable on this arm (2026-11-21), and direct goals make the contact stops (cancel → the controller holds) and the gripper preemption simple.
- **Clearance**: every transit segment is sampled against the table model (convex hulls of every link mesh and the wrist camera, `table_z` from `so101_kinematics.yaml`) plus a carried cube; the lowest point must stay ≥ `transit_clearance_mm` (cubes are 30 mm tall). Leaving or reaching a low pose only requires not going lower. Candidates: direct, via a pose `via_height` above the target, via a raised current pose; none clear → abort without moving.
- **Limits**: every arm goal is checked against the planning limits and |wrist_roll| ≤ `roll_limit_deg` before it is sent; IK bounds keep the roll inside as well.
- **Speeds** are parameters (`transit_speed`, `approach_speed`, `descend_speed`, `lift_speed`, gripper `close_speed_deg_s` / `open_speed_deg_s`).
- **Cancel** (Ctrl-C in the CLI, or cancel the goal): active controller goals are canceled, so the joints hold where they are; the gripper is never opened by a cancel. One goal at a time; others are rejected while busy.
- The gripper opens only in: approach (high above the table), a miss at the bottom (after "not held"), release, and the `open_gripper` service.

## Perception adapter (`cube_tracker.py`)

`CubeTracker.get(cube_id, since)` → `CubeEstimate(centre, yaw, stable, spread_mm, yaw_spread_deg, age_s, count)` is the only place the node reads perception. `cube_source`:

- `tracked`: `/cubes/tracked`, the detector's temporal filter (score ≥ 0.5 = stable, spread from the covariance, stamp = last image the cube was seen in). Stable for the node also needs `obs_min_fresh` new stamps after `since`; if the filter still lags (old far-view observations in its window), `obs_min_count` fresh observations that agree count as stable with their own median.
- `detections`: `/cubes/detections`, the node's own median window (`obs_*`).
- `auto` (default): tracked for a cube the filter reported in the last 2 s, else detections.

## Tests

```bash
cd so101_manipulation && python3 -m pytest -q test          # no ROS graph needed: geometry, soft grasp, adapter, torque hook, params
export ROS_DOMAIN_ID=42                                       # mock integration test, see its docstring
ros2 launch so101_moveit_config demo.launch.py rviz:=false &
python3 test/smoke_mock.py                                    # miss, pick_place, search, reach, cancel
```

`test/smoke_mock.py` starts its own FAKE cube publisher (`fake_cube_publisher`: `/cubes/detections` and `/cubes/tracked`, frames ignored while the arm moves, optional field-of-view cone and far-view bias) and node per scenario, and refuses to run on domain 0 or on real hardware.

## Files

| File | Content |
|---|---|
| `so101_manipulation/pick_place_node.py` | action server, phases, parameters (`PARAMS`) |
| `so101_manipulation/grasp.py` | grasp IK, closing options, reach, view poses |
| `so101_manipulation/soft_grasp.py` | floor, contact detector, hold target, held / empty |
| `so101_manipulation/kinematics.py` | FK from the URDF + `so101_kinematics.yaml`, mesh hulls, table clearance (no ROS) |
| `so101_manipulation/motion.py` | trajectories to the controllers, limit and floor checks, lag / current / torque-limit state |
| `so101_manipulation/cube_tracker.py` | perception adapter |
| `so101_manipulation/hooks.py` | gripper torque limit (publish until the readback matches) |
| `so101_manipulation/cli.py`, `fake_cube_publisher.py` | `so101_cube` CLI, mock cube |
| `config/pick_place.yaml` | every parameter with its default (kept equal to `PARAMS` by `test/test_params.py`) |

## Open points (real arm)

- `grasp_tip_height` 11 mm with `table_z` known to ~2.4 mm rms: the moving jaw hangs ~6 mm lower than the fixed tip at 30° open (lowest point ~4.7 mm above the table in the model).
- The descent contact thresholds come from the toy grasp; a paper cube may give a weaker shoulder_lift lag when a jaw lands on it.
- Close-up visibility of the cube at the hover with the jaws open, and the far/close accuracy, are from one session.
- Stacking (`place --on`) has not run on mock with two cubes or on the real arm.

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

**Grasp geometry**

| Parameter | Default | Meaning |
|---|---|---|
| `open_deg` | `30.0` | gripper opening for the approach (~53 mm gap) |
| `grasp_tip_height` | `0.011` | fixed jaw tip above the cube's bottom at the grasp |
| `hover_height` | `0.07` | fixed jaw tip above the cube's bottom at the hover |
| `hover_min_height` | `0.05` | lowest hover tried when the hover is out of reach |
| `descend_step` | `0.01` | height step of the descent |
| `gap_fraction` | `0.5` | where the cube centre goes between the fixed (0) and moving (1) jaw tip; 0.5 = gap midpoint |
| `allow_tilt` | `true` | allow a tilted approach (<= max_tilt_deg) beyond vertical reach |
| `max_tilt_deg` | `15.0` | largest jaw tilt of the fallback |
| `vertical_tol_deg` | `3.0` | jaws count as vertical within this |
| `ik_tolerance_mm` | `2.0` | largest IK position error accepted |
| `roll_limit_deg` | `90.0` | /wrist_roll/ never beyond this (camera cable) |
| `min_final_clearance_mm` | `3.0` | lowest point of the hand above the table at the grasp pose |

**Close-up refinement**

| Parameter | Default | Meaning |
|---|---|---|
| `refine_max_iter` | `2` | close-up refinements at the hover |
| `refine_min_shift_mm` | `2.0` | re-target when the close-up estimate moved more than this |
| `refine_max_shift_mm` | `40.0` | abort when the close-up estimate moved more than this |
| `refine_timeout_s` | `3.0` | wait this long for a stable close-up estimate |
| `refine_required` | `false` | abort when the close-up sees no stable cube (else keep the far estimate) |

**Contact on the way down (arm)**

| Parameter | Default | Meaning |
|---|---|---|
| `contact_joint` | `shoulder_lift` | joint whose lag (reference - feedback) shows a contact |
| `descend_contact_lag_deg` | `1.5` | lag above the hover baseline after a step = contact |
| `motion_contact_lag_deg` | `4.0` | lag above the hover baseline during a step = contact |
| `descend_lag_samples` | `15` | controller states averaged after each step |

**Soft grasp (gripper)**

| Parameter | Default | Meaning |
|---|---|---|
| `close_speed_deg_s` | `12.0` | closing ramp speed |
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
| `transit_speed` | `0.6` | rad/s, transit |
| `approach_speed` | `0.3` | rad/s, re-targeting at the hover |
| `descend_speed` | `0.12` | rad/s, descent steps |
| `lift_speed` | `0.2` | rad/s, lift / retreat |
| `min_move_s` | `0.8` | shortest move |
| `settle_s` | `0.3` | pause after each move |
| `transit_clearance_mm` | `45.0` | lowest point (arm, camera, carried cube) above the table on transits; cubes are 30 mm tall |
| `via_height` | `0.12` | tip height of the via pose above a target |
| `table_z` | `NaN` | table surface in base_link; NaN = so101_kinematics.yaml |
