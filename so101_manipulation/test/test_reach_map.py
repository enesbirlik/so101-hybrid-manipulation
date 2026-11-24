"""Reach table: summary per pan, failure classes, the yaml the node publishes on ~/reach."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_manipulation.reach_map import COLUMNS, classify, load_reach, merge_place, summarize, write_yaml  # noqa: E402,E501

ROOT = Path(__file__).resolve().parents[1]


def rows():
    out = []
    for r in [0.08, 0.09, 0.10, 0.11, 0.12, 0.40, 0.41, 0.42, 0.43]:
        for yaw in (0, 45):
            ok = (0.09 <= r <= 0.42) if yaw == 0 else (0.10 <= r <= 0.41)
            x = dict(th=0, r=r, yaw=yaw, ok=ok)
            if ok:
                x["lean"] = 45.0 if r > 0.3 else -10.0
            else:
                x["why"] = ("no grasp ...: vertical: close +0: needs roll +179 deg | tilted: close +90 lean +45: "
                            "grasp: position off by 5.0 mm (elbow_flex at its limit) | tilted: close +180 lean +45: "
                            "grasp: closing -46 deg off horizontal (max 20)")
            out.append(x)
    return out


def test_summary_bands():
    s = summarize(rows())[0]
    assert (s["near_m"], s["far_m"], s["near_all_m"], s["far_all_m"]) == (0.09, 0.42, 0.10, 0.41)
    assert s["lean_at_far"] == {"0": 45.0}
    assert "joint limit (elbow_flex)" in s["far_stop"]["0"] and "closing off horizontal" in s["far_stop"]["0"]


def test_classify():
    assert classify("x | tilted: close +45: needs roll -136 deg | tilted: close +135 lean +45: grasp: closing -35 deg "
                    "off horizontal (max 20)") == "wrist_roll limit / closing off horizontal"
    assert classify("tilted: a: grasp: lowest point 1.0 mm above the table") == "table clearance"
    assert classify("tilted: a: position off by 9 mm | tilted: b: position off by 3 mm") == "out of reach (IK) x2"


def test_yaml_roundtrip(tmp_path):
    p = tmp_path / "reach.yaml"
    write_yaml(p, summarize(rows()), {"max_tilt_deg": 45.0})
    r = load_reach(p)
    assert r["columns"] == COLUMNS and r["max_tilt_deg"] == 45.0
    row = r["rows"][0]
    assert row[:5] == [0.0, 0.09, 0.42, 0.10, 0.41] and math.isnan(row[5]) and math.isnan(row[6])


def test_place_band(tmp_path):
    """write_yaml with a vertical-jaw summary fills place_*; merge_place adds it to an old 5-column table."""
    place = [{"pan_deg": 0, "near_m": 0.12, "far_m": 0.31}]
    p = tmp_path / "reach.yaml"
    write_yaml(p, summarize(rows()), {"max_tilt_deg": 45.0}, place)
    assert load_reach(p)["rows"] == [[0.0, 0.09, 0.42, 0.10, 0.41, 0.12, 0.31]]
    old = tmp_path / "old.yaml"
    old.write_text("reach:\n  columns: [pan_deg, near_m, far_m, near_all_m, far_all_m]\n"
                   "  rows:\n  - [0.0, 0.09, 0.42, .nan, .nan]\n  max_tilt_deg: 45.0\n")
    merge_place(old, place)
    r = load_reach(old)
    assert r["max_tilt_deg"] == 45.0 and r["rows"][0][:3] == [0.0, 0.09, 0.42] and r["rows"][0][5:] == [0.12, 0.31]
    assert math.isnan(r["rows"][0][3])


def test_shipped_reach_table():
    """config/reach.yaml: one row per pan, near < far, all-yaw band inside the some-yaw band."""
    r = load_reach(ROOT / "config" / "reach.yaml")
    assert r["columns"] == COLUMNS and len(r["rows"]) >= 5
    for pan, near, far, near_all, far_all, place_near, place_far in r["rows"]:
        assert -90 <= pan <= 90
        assert 0.05 < near < far < 0.5
        if not math.isnan(near_all):
            assert near <= near_all <= far_all <= far
        # the place band (vertical jaws) lies inside the pick band (which may lean)
        assert near <= place_near < place_far <= far
        assert place_far <= 0.33, "a vertical hold cannot be put down beyond ~32 cm (2026-11-24: refused at 34 cm)"
