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
