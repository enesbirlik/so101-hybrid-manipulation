"""Synthetic check of cube_detection: a cube at a known pose is ray-traced through the wrist camera's
fisheye model, detected, and its centre and yaw compared with the truth.

    python3 -m pytest so101_perception/test/test_cube_detection.py -q
"""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, HERE)
import cube_detection as cd  # noqa: E402
import cube_sim as sim  # noqa: E402

camera = sim.camera


def render(cube_k, centre_w, yaw, R_wc, t_wc, K, D):
    """An ideal cube, sharp, no noise (the original test image)."""
    cube = sim.ideal_cube(cube_k, centre_w, yaw)
    return sim.render([cube], R_wc, t_wc, K, D, supersample=1, blur_sigma=0, jpeg_quality=0,
                      table_grey=120)


look_at = sim.look_at


@pytest.mark.parametrize("cube_k,yaw_deg,eye", [
    (0, 0.0, [0.20, 0.0, 0.25]),
    (3, 25.0, [0.15, 0.05, 0.22]),
    (5, -40.0, [0.30, -0.08, 0.18]),
])
def test_centre_and_yaw(cube_k, yaw_deg, eye):
    K, D = camera()
    centre = np.array([0.25, 0.03, cd.CUBE_EDGE / 2])
    yaw = np.radians(yaw_deg)
    R_wc, t_wc = look_at(np.array(eye), centre)
    gray = render(cube_k, centre, yaw, R_wc, t_wc, K, D)

    poses = cd.marker_poses(cd.MarkerDetector().detect(gray), K, D)
    free = cd.cube_estimates(poses, R_wc, t_wc)
    assert [e.cube for e in free] == [cube_k], f"detected {[p.marker_id for p in poses]}"
    assert np.linalg.norm(free[0].centre - centre) < 0.006, (free[0].centre, centre)
    # resting on the table: the height is known, the rest comes from the viewing rays
    e = cd.cube_estimates(poses, R_wc, t_wc, table_centre_z=centre[2])[0]
    assert np.linalg.norm(e.centre - centre) < 0.0025, (e.centre, centre)
    assert abs(cd.fold_yaw(e.yaw - yaw)) < np.radians(3)
    assert e.up_axis_tilt < np.radians(5)


def test_fold_yaw():
    for a in np.radians([0, 44, 46, 90, 135, -100, 181]):
        f = cd.fold_yaw(a)
        assert -np.pi / 4 - 1e-12 <= f < np.pi / 4
        assert abs(np.sin(4 * (f - a))) < 1e-9


def test_foreign_ids_ignored():
    assert cd.cube_of(35) == 5
    assert cd.cube_of(36) is None


# --- imperfect paper cubes ---------------------------------------------------------------

def test_paper_cubes_monte_carlo():
    """Random imperfect cubes (cube_sim.Imperfections: faces tilted 4 deg, markers 2 mm off-centre,
    rotated 3 deg, scaled 2 %, bulged) from random wrist poses. Targets of the grasp: centre within
    3 mm, yaw within 3 deg; per frame, from one or two faces, that holds for ~95 %."""
    import cube_monte_carlo as mc
    r = mc.run(n=80, seed=3)
    xy, yaw = r["xy"] * 1000, np.degrees(r["yaw"])
    assert len(xy) >= 0.9 * r["n"] and r["wrong"] == 0
    assert np.percentile(xy, 50) < 2.2 and np.percentile(xy, 95) < 3.8 and xy.max() < 8, mc.summary(r)
    assert np.percentile(yaw, 50) < 2.0 and np.percentile(yaw, 95) < 4.5 and yaw.max() < 10, mc.summary(r)


def test_ideal_cubes_are_sub_mm():
    """Without paper imperfections the same pipeline is well under the targets."""
    import cube_monte_carlo as mc
    r = mc.run(n=40, seed=4, imp=sim.Imperfections.none())
    xy, yaw = r["xy"] * 1000, np.degrees(r["yaw"])
    assert np.percentile(xy, 95) < 1.5 and np.percentile(yaw, 95) < 1.5, mc.summary(r)


def test_both_ippe_solutions_kept():
    K, D = camera()
    rng = np.random.default_rng(0)
    cube, R_wc, t_wc = sim.random_scene(rng, sim.Imperfections.none(), k=2)
    poses = cd.marker_poses(cd.MarkerDetector().detect(sim.render([cube], R_wc, t_wc, K, D, rng=rng)), K, D)
    assert poses
    for p in poses:
        assert 1 <= len(p.solutions) <= 2
        assert p.solutions[0][2] <= p.solutions[-1][2]
        assert p.bearings.shape == (4, 3) and np.allclose(np.linalg.norm(p.bearings, axis=1), 1)


def test_oblique_far_views():
    """Far, oblique views as from the folded rest pose (camera 15 cm up, cubes 30-40 cm out): the
    lower-error IPPE pose of a small top marker can be tilted the wrong way (normal pointing down).
    The upright cube must still come out right."""
    K, D = camera()
    rng = np.random.default_rng(11)
    detector = cd.MarkerDetector()
    found, bad_yaw = 0, 0
    for i in range(12):
        cube = sim.paper_cube(i % 6, [rng.uniform(0.30, 0.40), rng.uniform(-0.12, 0.05)],
                              rng.uniform(-np.pi, np.pi), 0.0, rng)
        R_wc, t_wc = sim.look_at([0.16, 0.06, 0.15], cube.centre, roll=np.pi / 2)
        gray = sim.render([cube], R_wc, t_wc, K, D, rng=rng)
        es = {e.cube: e for e in cd.estimate_cubes(detector.detect(gray), K, D, R_wc, t_wc,
                                                   table_centre_z=cube.centre[2])}
        e = es.get(cube.k)
        if e is None:
            continue
        found += 1
        assert np.linalg.norm(e.centre[:2] - cube.centre[:2]) < 0.006, (i, e.centre, cube.centre)
        bad_yaw += abs(np.degrees(sim.yaw_error(e.yaw, cube.yaw))) > 5
    assert found >= 10 and bad_yaw <= 1


def test_stacked_cube_not_snapped():
    """A cube 30 mm above the table prior (as if stacked; the cube below is left out) gets its height fitted, while a
    cube next to it on the table is snapped."""
    K, D = camera()
    rng = np.random.default_rng(5)
    lower = sim.ideal_cube(0, [0.27, -0.05, 0.015], 0.2)
    upper = sim.ideal_cube(1, [0.27, 0.03, 0.045], -0.3)
    R_wc, t_wc = sim.look_at([0.13, -0.01, 0.22], [0.27, -0.01, 0.03])
    gray = sim.render([lower, upper], R_wc, t_wc, K, D, rng=rng)
    es = {e.cube: e for e in cd.estimate_cubes(cd.MarkerDetector().detect(gray), K, D, R_wc, t_wc,
                                               table_centre_z=0.015)}
    assert es[0].on_table and not es[1].on_table
    assert abs(es[1].centre[2] - 0.045) < 0.004
    assert np.linalg.norm(es[1].centre[:2] - upper.centre[:2]) < 0.004


def test_cube_geometry_from_yaml(tmp_path):
    f = tmp_path / "cubes.yaml"
    f.write_text("default: {edge: 0.030, marker: 0.022}\ncubes:\n  3: {edge: 0.034}\n")
    g = cd.load_cube_geometry(str(f))
    assert cd.geometry_of(g, 0) == cd.CubeGeometry(0.030, 0.022)
    assert cd.geometry_of(g, 3) == cd.CubeGeometry(0.034, 0.022)
    assert cd.geometry_of(None, 3) == cd.CubeGeometry()
    shipped = cd.load_cube_geometry(os.path.join(HERE, "..", "config", "cubes.yaml"))
    assert shipped[None] == cd.CubeGeometry()


def test_per_cube_geometry_used():
    """A larger cube with a larger marker: right with its own geometry, off with the nominal one."""
    K, D = camera()
    rng = np.random.default_rng(6)
    big = sim.ideal_cube(4, [0.26, 0.02, 0.020], 0.3, edge=0.040, marker=0.030)
    R_wc, t_wc = sim.look_at([0.16, 0.0, 0.24], big.centre)
    det = cd.MarkerDetector().detect(sim.render([big], R_wc, t_wc, K, D, rng=rng))
    geometry = {None: cd.CubeGeometry(), 4: cd.CubeGeometry(0.040, 0.030)}
    right = cd.estimate_cubes(det, K, D, R_wc, t_wc, geometry=geometry, table_z=0.0)[0]
    assert np.linalg.norm(right.centre - big.centre) < 0.002
    nominal = cd.estimate_cubes(det, K, D, R_wc, t_wc, table_z=0.0)[0]
    assert np.linalg.norm(nominal.centre - big.centre) > 0.004


def test_confidence_and_spread_fields():
    K, D = camera()
    rng = np.random.default_rng(8)
    cube = sim.ideal_cube(5, [0.25, -0.03, 0.015], 0.4)
    R_wc, t_wc = sim.look_at([0.13, -0.10, 0.20], cube.centre)
    e = cd.estimate_cubes(cd.MarkerDetector().detect(sim.render([cube], R_wc, t_wc, K, D, rng=rng)),
                          K, D, R_wc, t_wc, table_centre_z=0.015)[0]
    assert len(e.faces) >= 2 and e.top_face >= 0
    assert 0.3 < e.confidence <= 1.0
    assert e.face_spread < 0.0015 and e.reproj_px < 1.0
    assert set(e.face_centres) == set(e.faces)


def test_circular_mean_90():
    m, r = cd.circular_mean_90(np.radians([44, -44]))
    assert abs(abs(np.degrees(m)) - 45) < 1e-6 and r > 0.99
    m, _ = cd.circular_mean_90(np.radians([10, 100, -80]))
    assert abs(np.degrees(m) - 10) < 1e-6
