"""Marker-less wooden cube detector: synthetic boxes through the real fisheye model, and the
segmentation on a real wrist frame."""

import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wood_sim as ws  # noqa: E402
import wood_detection as wd  # noqa: E402

REAL_FRAME = os.environ.get("WOOD_REAL_FRAME", os.path.join(HERE, "data", "wood_wrist.jpg"))
OVERLAY_OUT = os.environ.get("WOOD_OVERLAY_OUT")

# (x, y, yaw, a, b, h): 25, 35, 45 mm cubes and one irregular 40 x 35 x 38 box
SCENES = [
    [ws.Box(0.20, -0.07, 0.30, .025, .025, .025), ws.Box(0.25, 0.00, -0.50, .035, .035, .035),
     ws.Box(0.30, 0.08, 0.10, .045, .045, .045), ws.Box(0.19, 0.07, 0.70, .040, .035, .038)],
    [ws.Box(0.27, -0.06, -0.20, .045, .045, .045), ws.Box(0.21, 0.03, 0.40, .025, .025, .025),
     ws.Box(0.30, 0.05, 1.10, .040, .035, .038), ws.Box(0.17, -0.04, -0.9, .035, .035, .035)],
]


def _match(truth, boxes):
    d = [np.hypot(b.centre[0] - truth.x, b.centre[1] - truth.y) for b in boxes]
    return boxes[int(np.argmin(d))] if d and min(d) < 0.02 else None


def _truth_size(t, yaw_est):
    """Truth (a, b) in the frame of the folded estimated yaw."""
    k = int(round((t.yaw - yaw_est) / (np.pi / 2)))
    return np.array([t.b, t.a, t.h]) if k % 2 else np.array([t.a, t.b, t.h])


@pytest.mark.parametrize("scene", range(len(SCENES)))
def test_synthetic_boxes(scene):
    K, D = ws.camera()
    R_wc, t_wc = ws.overview_view()
    boxes = SCENES[scene]
    img = ws.render(boxes, R_wc, t_wc, K, D, rng=np.random.default_rng(scene))
    rays = wd.pixel_rays(K, D, ws.W, ws.H)
    good, bad, _ = wd.detect_boxes(img, R_wc, t_wc, K, D, 0.0, rays_cam=rays)
    assert len(good) == len(boxes), f"{len(good)} accepted, {len(bad)} rejected"
    for t in boxes:
        b = _match(t, good)
        assert b is not None, f"no detection near {t}"
        err_c = np.hypot(b.centre[0] - t.x, b.centre[1] - t.y)
        err_z = abs(b.centre[2] - t.h / 2)
        err_s = np.abs(b.size - _truth_size(t, b.yaw))
        err_yaw = abs(wd.fold_yaw(b.yaw - t.yaw))
        print(f"{t}: centre {err_c * 1000:.2f} mm, z {err_z * 1000:.2f} mm, size {np.round(err_s * 1000, 2)} mm, "
              f"yaw {np.degrees(err_yaw):.1f} deg, iou {b.iou:.3f}")
        assert err_c <= 0.003
        assert err_s.max() <= 0.003
        assert np.degrees(err_yaw) <= 6.0


def test_tracker_ids_and_sizes():
    tr = wd.WoodTracker()
    mk = lambda x, y, a: wd.WoodBox(np.array([x, y, a / 2]), 0.1, np.array([a, a, a]), 0.95, 0.5, 0.9)  # noqa: E731
    for i in range(8):
        n = 1e-4 * np.sin(i)
        out = tr.update(float(i), [mk(0.2 + n, 0.0, 0.030), mk(0.25, 0.05 + n, 0.045)])
    ids = sorted(k for k, _ in out)
    assert ids == [10, 11]
    assert all(tr.tracks[k].stable and tr.tracks[k].confidence >= 0.5 for k in ids)
    assert abs(tr.size(10)[0] - 0.030) < 1e-9
    # a new box far away gets a new id
    out = tr.update(9.0, [mk(0.35, -0.1, 0.03)])
    assert out[0][0] == 12


@pytest.mark.skipif(not os.path.exists(REAL_FRAME), reason="no real wrist frame")
def test_real_frame_segmentation():
    img = cv2.resize(cv2.imread(REAL_FRAME), (640, 360), interpolation=cv2.INTER_AREA)
    prm = wd.WoodParams()
    # pose unknown for this frame: the table is the right part of the image (desk edge at x ~ 375)
    roi = np.zeros(img.shape[:2], bool)
    roi[:, 380:] = True
    blobs = wd.split_blobs(wd.wood_mask(img, prm, roi), img, prm)
    assert len(blobs) >= 6
    # nothing on the orange jaws, even without the ROI (the desk and room are left to the work-area mask)
    full = wd.wood_mask(img, prm)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    jaws = cv2.morphologyEx(((hsv[..., 1] > 200) & (hsv[..., 0] < 20)).astype(np.uint8), cv2.MORPH_CLOSE,
                            np.ones((9, 9), np.uint8)) > 0
    jaws[180:] = False          # the jaws are in the upper part of this frame
    print(f"blobs {len(blobs)}, jaw pixels {jaws.sum()}, of them wood {np.count_nonzero(full[jaws])}")
    assert jaws.sum() > 5000
    assert np.count_nonzero(full[jaws]) < 0.01 * jaws.sum()
    for b in blobs:
        x, y, w, h = cv2.boundingRect(b)
        assert x >= 380
    if OVERLAY_OUT:
        out = img.copy()
        for i, b in enumerate(blobs):
            cnts, _ = cv2.findContours(b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(out, cnts, -1, [(0, 255, 0), (255, 0, 255), (255, 255, 0)][i % 3], 2)
        cv2.imwrite(OVERLAY_OUT, out)
