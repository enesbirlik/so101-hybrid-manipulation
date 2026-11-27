"""Wooden cubes of different sizes: per-size soft-grasp params, tracker sizes, stack heights (no ROS)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation import soft_grasp as sg  # noqa: E402
from so101_manipulation.cube_tracker import (CubeWindow, TrackedState, edge_across, median_size,  # noqa: E402
                                             parse_size, stack_surface)

BASE = sg.SoftGraspParams(closing_speed_deg_s=6.0)
PAPER = sg.GripProfile("paper", 200.0, 1.5, 1.5, 6.0, 0.0)
WOOD = sg.GripProfile("wood", 450.0, 4.0, 4.0, 8.0, 3.0)


# --- per-size soft-grasp params --------------------------------------------------------------

@pytest.mark.parametrize("width", [25.0, 30.0, 45.0])
@pytest.mark.parametrize("prof", [PAPER, WOOD], ids=["paper", "wood"])
def test_floor_never_below_width_minus_squeeze(width, prof):
    P = sg.params_for_width(BASE, width, prof)
    floor = sg.floor_deg(P)
    # the floor's gap is exactly width - squeeze_mm, never narrower
    assert sg.gap_mm(floor, P) == pytest.approx(width - prof.squeeze_mm)
    assert sg.gap_mm(floor, P) >= width - sg.MAX_SQUEEZE_MM - 1e-9
    expected = sg.expected_contact_deg(width, P.gap_zero_mm, P.gap_per_deg_mm)
    assert floor < expected
    ramp, _ = sg.close_ramp(30.0 if width < 40 else 42.0, expected, P, 20.0, 3.5)
    assert all(a >= floor - 1e-9 for a, _ in ramp) and ramp[-1][0] == pytest.approx(floor)
    traj = sg.ramp_trajectory(30.0 if width < 40 else 42.0, ramp, P)
    assert all(a >= floor - 1e-9 for a, _, _ in traj)
    # hold target: contact - squeeze_deg, clamped to the floor (contacts at and below the expected angle)
    for contact in (expected + 1.0, expected, floor + 0.5, floor):
        assert sg.hold_target_deg(contact, P) >= floor - 1e-9


def test_paper_30mm_unchanged():
    """The paper profile at the nominal edge gives the old params (ArUco behaviour unchanged)."""
    P = sg.params_for_width(BASE, 30.0, PAPER)
    assert sg.floor_deg(P) == pytest.approx(sg.floor_deg(sg.SoftGraspParams()))
    assert P.window_deg == BASE.window_deg and P.squeeze_deg == 1.5 and P.closing_speed_deg_s == 6.0
    assert sg.approach_open_deg(30.0, 30.0, 11.5, 55.0) == 30.0


def test_wood_profile():
    P = sg.params_for_width(BASE, 30.0, WOOD)
    assert P.squeeze_deg == 4.0 and P.squeeze_mm == 4.0 and P.closing_speed_deg_s == 8.0
    assert P.window_deg == pytest.approx(BASE.window_deg + 3.0 / 1.35)
    # floor: 26 mm gap -> 9.6 deg
    assert sg.floor_deg(P) == pytest.approx((26.0 - 13.0) / 1.35)
    # a profile asking for a huge squeeze is capped
    P2 = sg.params_for_width(BASE, 30.0, sg.GripProfile("wood", 450, 4, 20, 8, 3))
    assert sg.gap_mm(sg.floor_deg(P2), P2) == pytest.approx(30.0 - sg.MAX_SQUEEZE_MM)


def test_approach_opening_by_size():
    assert sg.approach_open_deg(25.0, 30.0, 11.5, 55.0) == 30.0                  # open_deg is the minimum
    a45 = sg.approach_open_deg(45.0, 30.0, 11.5, 55.0)
    assert sg.gap_mm(a45, BASE) == pytest.approx(45.0 + 23.0)                   # 40.7 deg
    assert sg.approach_open_deg(80.0, 30.0, 11.5, 55.0) == 55.0                  # clamped to the max


def test_measured_width_from_contact():
    P = sg.params_for_width(BASE, 45.0, WOOD)
    contact = sg.expected_contact_deg(42.0)
    assert sg.gap_mm(contact, P) == pytest.approx(42.0)
    assert sg.in_window(contact, sg.expected_contact_deg(45.0), P)


# --- tracker sizes --------------------------------------------------------------------------

def test_parse_size():
    assert parse_size(0.0, 0.0, 0.0) is None
    assert parse_size(0.03, float("nan"), 0.03) is None
    assert np.allclose(parse_size(0.045, 0.04, 0.05), [0.045, 0.04, 0.05])


def test_window_median_size_and_default():
    w = CubeWindow(default_edge=0.03)
    for i in range(8):
        w.add(i * 0.1, [0.22, 0.03, 0.025], 0.1, 1.0, [0.045 + (0.004 if i == 3 else 0.0), 0.040, 0.050])
    e = w.estimate("cube_11", now=0.8)
    assert np.allclose(e.size, [0.045, 0.040, 0.050])
    w2 = CubeWindow(default_edge=0.03)
    for i in range(6):
        w2.add(i * 0.1, [0.22, 0.03, 0.015], 0.0)              # no size (old detector / missing bbox)
    assert np.allclose(w2.estimate("cube_2", now=0.6).size, [0.03, 0.03, 0.03])
    assert np.allclose(median_size([None], 0.025), [0.025] * 3)


def test_tracked_state_size():
    st = TrackedState(default_edge=0.03)
    for i in range(6):
        st.add(i * 0.1, i * 0.1, [0.22, 0.03, 0.02], 0.0, 0.6, 1.0, 1.0, size=[0.04, 0.04, 0.04])
    assert np.allclose(st.estimate("cube_12", now=0.6).size, [0.04, 0.04, 0.04])


def test_edge_across_closing_direction():
    size = [0.045, 0.030, 0.040]          # x along the yaw axis
    yaw = math.radians(20)
    assert edge_across(size, yaw, yaw) == 0.045
    assert edge_across(size, yaw, yaw + math.pi) == 0.045
    assert edge_across(size, yaw, yaw + math.pi / 2) == 0.030
    assert edge_across(size, yaw, yaw - math.pi / 2) == 0.030


# --- stack heights with mixed sizes ------------------------------------------------------------

TZ = 0.01


def test_stack_nominal_snaps_to_levels():
    # paper 30 mm cube on another, detector 7 mm high: snaps to level 2
    s, how = stack_surface([0.2, 0, TZ + 0.045 + 0.007], [0.03] * 3, TZ, 0.03, 0.012)
    assert s == pytest.approx(0.06) and how == "level 2"


def test_stack_wood_on_table_uses_measured_height():
    # 45 mm wooden cube on the table, centre z 2 mm off: top = its height
    s, how = stack_surface([0.2, 0, TZ + 0.0225 + 0.002], [0.045, 0.044, 0.045], TZ, 0.03, 0.012)
    assert s == pytest.approx(0.045) and "table" in how


def test_stack_wood_mixed_sizes():
    # a 25 mm cube on a 45 mm one: no whole-cube levels
    c = [0.2, 0, TZ + 0.045 + 0.0125]
    s, how = stack_surface(c, [0.025, 0.025, 0.025], TZ, 0.03, 0.012)
    assert s == pytest.approx(0.070) and how.startswith("measured")
    # placed by the node: the top it was placed at wins (xy within 10 mm)
    s, how = stack_surface(c, [0.025, 0.025, 0.025], TZ, 0.03, 0.012, placed=(np.array([0.205, 0.0]), 0.0705))
    assert s == pytest.approx(0.0705) and "placed" in how
    s, _ = stack_surface(c, [0.025, 0.025, 0.025], TZ, 0.03, 0.012, placed=(np.array([0.25, 0.0]), 0.0705))
    assert s == pytest.approx(0.070)
    # release height = lower top + held cube's grasp tip height (+ release gap): the held cube's bottom
    # lands on the lower cube's top whatever its own height
    grasp_h = min(0.011, 0.45 * 0.040)
    release_h = s + grasp_h + 0.002
    assert release_h - grasp_h == pytest.approx(0.072)
