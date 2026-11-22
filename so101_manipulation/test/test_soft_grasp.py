"""Soft grasp logic on injected gripper states (no ROS, no hardware)."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation import soft_grasp as sg  # noqa: E402

P = sg.SoftGraspParams()
EXPECTED = sg.expected_contact_deg(30.0)
FLOOR = sg.floor_deg(P)


def simulate(block_deg=None, start=30.0, speed=12.0, rate=100.0, free_lag=0.4, P_=P, play=0.0, noise=0.1, seed=0):
    """Commanded ramp from `start` down to the floor; the servo trails the command by `free_lag`
    (controller lag commanded - measured = -free_lag, as on the real arm) and is blocked at
    block_deg (a soft cube gives a little `play`). Returns (event, detector, last command)."""
    rng = np.random.default_rng(seed)
    det = sg.ContactDetector(P_, static_baseline_lag=0.0)
    end = sg.ramp_end_deg(EXPECTED, P_)
    t, cmd = 0.0, start
    while True:
        cmd = max(end, start - speed * t)
        free = cmd + free_lag
        meas = free if block_deg is None else max(free, block_deg - play * max(0.0, block_deg - cmd) / 10)
        ev = det.update(t, cmd, meas + rng.normal(0, noise))
        if ev:
            return ev, det, cmd
        if cmd <= end and t > (start - end) / speed + 0.5:
            return None, det, cmd
        t += 1 / rate


# --- the hard floor ------------------------------------------------------------------------

def test_floor_for_a_30mm_cube():
    """gap = 13 + 1.35 deg; floor = angle for 30 - 1.5 mm = 11.48 deg."""
    assert abs(FLOOR - (28.5 - 13.0) / 1.35) < 1e-9
    assert 11.3 < FLOOR < 11.6


def test_floor_scales_with_cube_and_squeeze():
    assert sg.floor_deg(sg.SoftGraspParams(edge_mm=40.0)) > FLOOR
    assert sg.floor_deg(sg.SoftGraspParams(squeeze_mm=0.0)) > FLOOR


def test_ramp_ends_at_the_floor_never_lower():
    assert sg.ramp_end_deg(EXPECTED, P) == FLOOR
    assert sg.ramp_end_deg(EXPECTED, P) > P.empty_deg
    # whatever the window: the ramp end does not depend on it
    assert sg.ramp_end_deg(EXPECTED, sg.SoftGraspParams(window_deg=20.0)) == FLOOR


def test_hold_target_clamped_to_the_floor():
    assert sg.hold_target_deg(13.5, P) == 13.5 - P.squeeze_deg
    assert sg.hold_target_deg(12.0, P) == FLOOR             # 12.0 - 1.5 would be below the floor
    assert sg.hold_target_deg(-9.0, P) == FLOOR
    big = sg.SoftGraspParams(squeeze_deg=10.0)
    assert sg.hold_target_deg(13.5, big) == sg.floor_deg(big)


def test_clamp_command():
    for cmd in (-10.0, 0.0, 5.0, 11.0):
        assert sg.clamp_command(cmd, FLOOR) == FLOOR
    assert sg.clamp_command(30.0, FLOOR) == 30.0


def test_motion_layer_clamp():
    f = math.radians(FLOOR)
    pts, clamped = sg.clamp_gripper_points([[math.radians(-10.0)], [math.radians(30.0)]], f)
    assert clamped and pts[0][0] == f and pts[1][0] == math.radians(30.0)
    pts, clamped = sg.clamp_gripper_points([[math.radians(20.0)]], f)
    assert not clamped
    pts, clamped = sg.clamp_gripper_points([[math.radians(-10.0)]], None)
    assert not clamped and pts[0][0] == math.radians(-10.0)


def test_no_command_in_a_whole_grasp_goes_below_the_floor():
    ev, det, last_cmd = simulate(block_deg=None)
    assert last_cmd >= FLOOR - 1e-9
    ev, det, last_cmd = simulate(block_deg=13.5)
    assert last_cmd >= FLOOR and sg.hold_target_deg(ev.angle_deg, P) >= FLOOR


# --- contact sign ------------------------------------------------------------------------------

def test_contact_sign_real_arm_numbers():
    """Real arm: controller lag (commanded - measured) -0.3..-0.5 free, -1.2 at contact, growing to -8.
    That is contact (blocked = measured - commanded rises)."""
    det = sg.ContactDetector(P, static_baseline_lag=0.4)
    t, cmd, ev = 0.0, 30.0, None
    while cmd > 13.6:                              # free closing, controller lag -0.4
        assert det.update(t, cmd, cmd + 0.4) is None
        t += 0.01
        cmd -= 0.12
    for k in range(40):                            # blocked at 13.5: controller lag grows negative
        ev = det.update(t, cmd, 13.5)
        if ev:
            break
        t += 0.01
        cmd -= 0.12
    assert ev is not None and ev.by == "lag"
    assert abs(ev.angle_deg - 13.5) < 1e-9
    assert -(ev.commanded_deg - ev.angle_deg) > 1.0      # controller lag below -1 deg at detection
    assert 13.5 - ev.commanded_deg < 2.5                 # detected before the command ran far past


def test_measured_below_command_is_never_contact():
    """Overshoot (measured below the command, controller lag positive) must not count."""
    det = sg.ContactDetector(P, static_baseline_lag=0.0)
    for i in range(200):
        cmd = 30.0 - 0.12 * i
        assert det.update(i * 0.01, cmd, cmd - 3.0) is None


def test_expected_contact_for_a_30mm_cube():
    assert abs(EXPECTED - 12.6) < 0.1


def test_contact_detected_close_to_the_cube_face():
    ev, _, _ = simulate(block_deg=13.5)
    assert ev is not None
    assert abs(ev.angle_deg - 13.5) < 0.5
    assert sg.in_window(ev.angle_deg, EXPECTED, P)


def test_no_false_contact_on_a_free_ramp():
    for seed in range(5):
        ev, _, _ = simulate(block_deg=None, seed=seed)
        assert ev is None


def test_tracking_lag_alone_is_not_contact():
    """A servo that trails the ramp by 2 deg steadily is not blocked (the running baseline absorbs it)."""
    ev, _, _ = simulate(block_deg=None, free_lag=2.0)
    assert ev is None


def test_soft_cube_with_play_still_detected():
    ev, _, _ = simulate(block_deg=13.5, play=1.0)
    assert ev is not None and abs(ev.angle_deg - 13.5) < 1.5


def test_contact_far_outside_the_window_is_flagged():
    ev, _, _ = simulate(block_deg=25.0)       # something wide between the jaws (cube edge, two cubes)
    assert ev is not None
    assert not sg.in_window(ev.angle_deg, EXPECTED, P)


def test_load_threshold_optional():
    """Real arm: load ~4 % free, 6-10 % right after contact."""
    p = sg.SoftGraspParams(load_pct=1.5, samples=2)
    det = sg.ContactDetector(p, static_baseline_lag=0.0, static_baseline_load=4.0)
    assert det.update(0.0, 20.0, 20.3, load=4.2) is None
    assert det.update(0.01, 19.9, 20.2, load=6.5) is None
    ev = det.update(0.02, 19.8, 20.1, load=7.0)
    assert ev is not None and ev.by == "load"


# --- held ----------------------------------------------------------------------------------------

def test_held_check_real_arm_numbers():
    """Real arm, 30 mm paper cube, command at the floor 11.48 deg: empty jaws stop at 12.04 (6 mA);
    with the cube at torque limit 20 %: 12.48-12.57 deg, 13-20 mA."""
    cmd = FLOOR
    assert sg.grip_state(12.04, cmd, P, 6.0)[0] == sg.EMPTY
    assert sg.grip_state(12.10, cmd, P, 6.0)[0] == sg.EMPTY
    assert sg.grip_state(12.48, cmd, P, 13.0)[0] == sg.HELD
    assert sg.grip_state(12.57, cmd, P, 20.0)[0] == sg.HELD
    assert sg.grip_state(12.48, cmd, P, float("nan"))[0] == sg.HELD     # angle alone (no current: mock)
    assert sg.grip_state(12.10, cmd, P, 15.0)[0] == sg.HELD             # current alone
    assert sg.grip_state(12.25, cmd, P, 8.0)[0] == sg.UNCERTAIN         # in between: not held
    assert sg.grip_state(11.48, cmd, P, float("nan"))[0] == sg.EMPTY    # mock: no deadband, at the command
    assert not sg.held(12.25, cmd, P, 8.0)[0]
    assert sg.held(12.5, cmd, P)[0]


def test_held_relative_to_the_hold_command():
    hold = 13.0                      # a hold target above the floor (contact seen early)
    assert sg.grip_state(13.56, hold, P, 5.0)[0] == sg.EMPTY
    assert sg.grip_state(14.0, hold, P, 5.0)[0] == sg.HELD


def test_simulated_measurement():
    assert sg.simulated_measurement(5.0, 5.0, float("nan")) == 5.0
    assert sg.simulated_measurement(12.0, 12.0, 13.5) == 13.5
    assert sg.simulated_measurement(30.0, 30.0, 13.5) == 30.0


def test_ramp_start_lag_above_the_window_is_not_contact():
    """Real arm, 2026-11-22, torque limit 20 %: right after the close starts at 29.6 deg the jaw
    trails the ramp by > 1 deg; that counted as contact at 27 deg. Above expected + window the lag
    only trains the baseline; a jaw blocked at 13.5 deg is still caught."""
    expected = sg.expected_contact_deg(30.0)
    det = sg.ContactDetector(P, static_baseline_lag=0.0, ignore_above_deg=expected + P.window_deg)
    t, cmd, meas = 0.0, 29.6, 29.6
    while cmd > 13.6:
        lag = 2.5 if cmd > 26.0 else 0.4           # start-up trailing, then steady tracking
        assert det.update(t, cmd, cmd + lag) is None
        t += 0.01
        cmd -= 0.06
    ev = None
    for _ in range(100):
        ev = det.update(t, cmd, 13.5)
        if ev:
            break
        t += 0.01
        cmd -= 0.06
    assert ev is not None and abs(ev.angle_deg - 13.5) < 1e-9


def test_jaws_blocked_above_the_window_still_seen():
    """Something wide between the jaws stops them at 22 deg: once the command passes the window the
    gap is large, so the detector reports it (the node then aborts as 'early')."""
    expected = sg.expected_contact_deg(30.0)
    det = sg.ContactDetector(P, static_baseline_lag=0.4, ignore_above_deg=expected + P.window_deg)
    t, cmd, ev = 0.0, 29.6, None
    while cmd > 11.5 and ev is None:
        ev = det.update(t, cmd, max(cmd + 0.4, 22.0))
        t += 0.01
        cmd -= 0.06
    assert ev is not None and ev.angle_deg > expected + P.window_deg


# --- two-speed close (fast above the window, slow inside it) --------------------------------

P6 = sg.SoftGraspParams(closing_speed_deg_s=6.0)


def ramp_command(pts, t, start):
    """Linear interpolation of close_ramp points (the controller gets no inner velocities)."""
    a0, t0 = start, 0.0
    for a, t1 in pts:
        if t <= t1:
            return a0 + (a - a0) * (t - t0) / max(1e-9, t1 - t0)
        a0, t0 = a, t1
    return pts[-1][0]


def test_close_ramp_never_below_the_floor_and_ends_there():
    floor = sg.floor_deg(P6)
    for start in (30.0, 25.0, 18.0, 12.0, 11.0, 5.0):
        for fast in (0.0, 6.0, 20.0, 60.0):
            pts, slow_from = sg.close_ramp(start, EXPECTED, P6, fast, 3.5)
            assert all(a >= floor for a, _ in pts), (start, fast, pts)
            assert pts[-1][0] == floor
            assert [t for _, t in pts] == sorted(t for _, t in pts)
            for t in np.linspace(0, pts[-1][1] + 1, 300):
                assert ramp_command(pts, t, max(start, floor)) >= floor - 1e-9


def test_close_ramp_is_slow_inside_the_window():
    pts, slow_from = sg.close_ramp(30.0, EXPECTED, P6, 20.0, 3.5)
    top = EXPECTED + P6.window_deg
    assert slow_from >= top + 3.5 - 1e-9
    assert len(pts) == 2 and pts[0][0] == slow_from
    # fast part: only above slow_from
    assert abs(pts[0][1] - (30.0 - slow_from) / 20.0) < 1e-9
    # slow part: closing_speed_deg_s from slow_from to the floor (covers the whole window)
    v = (pts[0][0] - pts[1][0]) / (pts[1][1] - pts[0][1])
    assert abs(v - 6.0) < 1e-9
    # sampled: every command at or below the window top moves at <= the closing speed
    dt = 0.01
    prev = None
    for t in np.arange(0, pts[-1][1] + 0.2, dt):
        c = ramp_command(pts, t, 30.0)
        if prev is not None and prev <= top + 1e-9:
            assert (prev - c) / dt <= 6.0 + 1e-6
        prev = c
    # and it is faster than the single slow ramp
    single, _ = sg.close_ramp(30.0, EXPECTED, P6, 0.0)
    assert pts[-1][1] < single[-1][1] - 0.8


def test_close_ramp_starting_in_the_window_is_one_slow_ramp():
    pts, slow_from = sg.close_ramp(16.0, EXPECTED, P6, 20.0, 3.5)
    assert len(pts) == 1 and slow_from == 16.0
    assert abs(pts[0][1] - (16.0 - sg.floor_deg(P6)) / 6.0) < 1e-9


def simulate_two_speed(block_deg, fast=20.0, rate=100.0, k_lag=0.08, tau=0.15, noise=0.05, seed=1):
    """Two-speed command; the jaw trails it by k_lag * speed (first order, tau): 1.6 deg in the
    fast part, 0.5 in the slow one; blocked at block_deg (None: empty jaws)."""
    rng = np.random.default_rng(seed)
    pts, slow_from = sg.close_ramp(30.0, EXPECTED, P6, fast, 3.5)
    det = sg.ContactDetector(P6, static_baseline_lag=0.0, ignore_above_deg=EXPECTED + P6.window_deg,
                             baseline_below_deg=slow_from + 1e-6)
    t, lag, prev = 0.0, 0.0, 30.0
    while t < pts[-1][1] + 1.0:
        c = ramp_command(pts, t, 30.0)
        speed = (prev - c) * rate
        lag += (k_lag * speed - lag) / (tau * rate)
        meas = c + lag
        if block_deg is not None:
            meas = max(meas, block_deg)
        ev = det.update(t, c, meas + rng.normal(0, noise))
        if ev:
            return ev, c
        prev = c
        t += 1 / rate
    return None, prev


def test_two_speed_close_no_false_contact_when_empty():
    ev, last = simulate_two_speed(None)
    assert ev is None and abs(last - sg.floor_deg(P6)) < 1e-6


def test_two_speed_close_detects_the_cube_near_its_face():
    for block in (13.5, 15.0, 16.5):   # 12.6 is too close to the floor for any ramp (held check)
        ev, cmd = simulate_two_speed(block)
        assert ev is not None, block
        assert abs(ev.angle_deg - block) < 0.3
        _, cmd_single = simulate_two_speed(block, fast=0.0)
        assert abs(cmd - cmd_single) < 0.3, (block, cmd, cmd_single)   # as early as the slow ramp alone


def hermite(p0, v0, p1, v1, T, s):
    """Cubic Hermite position at s in [0, T] (what joint_trajectory_controller interpolates)."""
    x = s / T
    h00, h10, h01, h11 = 2 * x**3 - 3 * x**2 + 1, x**3 - 2 * x**2 + x, -2 * x**3 + 3 * x**2, x**3 - x**2
    return h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1


def test_ramp_trajectory_cubic_never_below_the_floor_and_slow_in_the_window():
    floor = sg.floor_deg(P6)
    top = EXPECTED + P6.window_deg
    for start in (30.0, 22.0, 16.0, 12.0, 11.48, 5.0):
        for fast in (0.0, 20.0):
            pts, _ = sg.close_ramp(start, EXPECTED, P6, fast, 3.5)
            traj = sg.ramp_trajectory(start, pts, P6)
            assert traj[-1][0] == floor and traj[-1][1] == 0.0
            assert all(a >= floor for a, _, _ in traj)
            prev_p, prev_v, prev_t = max(start, floor), 0.0, 0.0
            last_pos = prev_p
            for a, v, t in traj:
                T = t - prev_t
                assert T > 0
                for s in np.linspace(0, T, 60):
                    pos = hermite(prev_p, prev_v, a, v, T, s)
                    assert pos >= floor - 1e-9, (start, fast, pos)
                    assert pos <= last_pos + 1e-9            # closing only, never back open
                    if last_pos <= top and s > 0:
                        assert (last_pos - pos) / (T / 59) <= 6.0 * 1.01, (start, fast, last_pos, pos)
                    last_pos = pos
                prev_p, prev_v, prev_t = a, v, t


def test_fast_part_does_not_train_the_baseline():
    """Samples commanded above baseline_below_deg are not in the running baseline."""
    det = sg.ContactDetector(P6, baseline_below_deg=20.0)
    for k in range(100):
        det.update(k * 0.01, 25.0 - k * 0.04, 25.0 - k * 0.04 + 2.0)   # trailing 2 deg, above 20
    assert len(det.hist) == 0
    det.update(1.0, 19.0, 19.5)
    assert len(det.hist) == 1
