"""config/pick_place.yaml carries every node parameter with the same default as PARAMS."""

import ast
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def node_params():
    src = (ROOT / "so101_manipulation" / "pick_place_node.py").read_text()
    start = src.index("PARAMS = {")
    end = src.index("\n}\n", start) + 2
    tree = ast.parse(src[start:end])
    out = {}
    for k, v in zip(tree.body[0].value.keys, tree.body[0].value.values):
        d = v.elts[0]
        out[k.value] = float("nan") if isinstance(d, ast.Name) and d.id == "NAN" else ast.literal_eval(d)
    return out


def test_yaml_matches_defaults():
    y = yaml.safe_load((ROOT / "config" / "pick_place.yaml").read_text())["cube_pick_place"]["ros__parameters"]
    p = node_params()
    assert set(y) == set(p)
    for k, v in p.items():
        if isinstance(v, float) and math.isnan(v):
            assert math.isnan(y[k]), k
        else:
            assert y[k] == v, k
