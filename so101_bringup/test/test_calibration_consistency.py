"""Offline consistency of the calibration, URDF, MoveIt, SRDF and controller files.

so101_hardware clamps commands to the calibrated servo range, so a goal that URDF or MoveIt allows
but the calibration does not would be clipped silently. These tests read the source tree (no colcon
install, no ROS) and fail when the files drift apart.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM = JOINTS[:5]
TOL = 1e-6
URDF = "so101_description/urdf/so101_new_calib.urdf"


def load_yaml(rel):
    return yaml.safe_load((REPO / rel).read_text())


def deg(rad):
    return math.degrees(rad)


def urdf_joints():
    root = ET.parse(REPO / URDF).getroot()
    return [j for j in root.iter("joint") if j.get("type") not in (None, "fixed")]


def urdf_limits():
    return {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in urdf_joints()}


def calib():
    return load_yaml("so101_bringup/config/so101_calibration.yaml")["joints"]


def calib_range_rad(c):
    a, b = (c["direction"] * (c[k] - c["zero"]) * 2 * math.pi / 4096 for k in ("range_min", "range_max"))
    return min(a, b), max(a, b)


def moveit_limits():
    jl = load_yaml("so101_moveit_config/config/joint_limits.yaml")["joint_limits"]
    return {n: (v["min_position"], v["max_position"]) for n, v in jl.items()}


def srdf_states():
    root = ET.parse(REPO / "so101_moveit_config/config/so101.srdf").getroot()
    return [(g.get("name"), j.get("name"), float(j.get("value")))
            for g in root.iter("group_state") for j in g.iter("joint")]


# Known exception: joint_limits.yaml lines 1-4 (the folded rest reads up to 92.3 deg on its stop).
ELBOW_UPPER = pytest.mark.xfail(
    strict=True,
    reason="elbow_flex upper: documented exception (joint_limits.yaml header), above the calibrated "
    "range_max: MoveIt 1.62 rad = 92.8 deg, URDF 1.69 rad = 96.8 deg, calibrated 1.6015 rad = 91.8 deg")
SIDES = [pytest.param(j, s, marks=[ELBOW_UPPER] if (j, s) == ("elbow_flex", 1) else [])
         for j in JOINTS for s in (0, 1)]


def test_joint_names_match_everywhere():
    xacro = ET.parse(REPO / "so101_description/urdf/so101.ros2_control.xacro").getroot()
    sets = {
        "calibration": set(calib()),
        "urdf (non-fixed)": {j.get("name") for j in urdf_joints()},
        "ros2_control xacro": {v for v in [e.get("name" if e.tag == "joint" else "joint") for e in xacro.iter()]
                               if v and not v.startswith("$")},
        "joint_limits.yaml": set(moveit_limits()),
    }
    for name, s in sets.items():
        assert s == set(JOINTS), f"{name}: joints {sorted(s)} differ from {JOINTS}"
    ctl = load_yaml("so101_bringup/config/ros2_controllers.yaml")
    assert ctl["arm_controller"]["ros__parameters"]["joints"] == ARM
    assert ctl["gripper_controller"]["ros__parameters"]["joints"] == ["gripper"]
    ids = [v["id"] for v in calib().values()]
    assert len(set(ids)) == len(ids), f"servo ids not unique: {ids}"


@pytest.mark.parametrize("joint", JOINTS)
def test_calibration_range_and_direction(joint):
    c = calib()[joint]
    assert c["range_min"] < c["zero"] < c["range_max"], (
        f"{joint}: need range_min < zero < range_max, got {c['range_min']}, {c['zero']}, {c['range_max']}")
    assert c["direction"] in (1, -1), f"{joint}: direction {c['direction']} is not +1 or -1"


@pytest.mark.parametrize("joint", JOINTS)
def test_moveit_limits_within_urdf(joint):
    lo, hi = moveit_limits()[joint]
    ulo, uhi = urdf_limits()[joint]
    assert lo >= ulo - TOL and hi <= uhi + TOL, (
        f"{joint}: MoveIt [{deg(lo):.2f}, {deg(hi):.2f}] deg outside URDF [{deg(ulo):.2f}, {deg(uhi):.2f}] deg")


def check_inside_range(joint, lim, side, what):
    rlo, rhi = calib_range_rad(calib()[joint])
    ok = lim[0] >= rlo - TOL if side == 0 else lim[1] <= rhi + TOL
    assert ok, (
        f"{joint}: {what} {'lower' if side == 0 else 'upper'} {deg(lim[side]):.2f} deg is outside the "
        f"calibrated range [{deg(rlo):.2f}, {deg(rhi):.2f}] deg; so101_hardware would clip it")


@pytest.mark.parametrize("joint,side", SIDES)
def test_moveit_limits_within_calibrated_range(joint, side):
    check_inside_range(joint, moveit_limits()[joint], side, "MoveIt")


@pytest.mark.parametrize("joint,side", SIDES)
def test_urdf_limits_within_calibrated_range(joint, side):
    check_inside_range(joint, urdf_limits()[joint], side, "URDF")


def test_srdf_group_states_within_moveit_limits():
    lims = moveit_limits()
    states = srdf_states()
    assert states, "no group_state joints found in the SRDF"
    for state, joint, v in states:
        lo, hi = lims[joint]
        assert lo - TOL <= v <= hi + TOL, (
            f"group_state '{state}': {joint} = {deg(v):.2f} deg outside MoveIt [{deg(lo):.2f}, {deg(hi):.2f}] deg")


def test_kinematics_yaml():
    k = load_yaml("so101_description/config/so101_kinematics.yaml")
    z = k["table_z"]
    assert isinstance(z, float) and -0.05 <= z <= 0, f"table_z = {z} m, expected a float in [-0.05, 0]"
    names = {j.get("name") for j in urdf_joints()}
    assert k["joints"], "no joints in so101_kinematics.yaml"
    for name, v in k["joints"].items():
        assert name in names, f"kinematics joint '{name}' is not a non-fixed joint in the URDF"
        for key in ("xyz", "rpy"):
            assert len(v.get(key, [])) == 3, f"{name}: {key} must have 3 entries, got {v.get(key)}"
