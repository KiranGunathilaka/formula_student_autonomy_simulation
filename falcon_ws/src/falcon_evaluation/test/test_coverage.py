"""Coverage ray-cast against brute-force ray casting (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_evaluation.coverage import CONE_SHAPES, CoverageAccumulator, _ray_frustum


def _pitch(p):
    c, s = math.cos(p), math.sin(p)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _brute(channels, hfov, n_h, O, R, cones):
    el = np.radians(channels)
    az = (-np.pi + 2 * np.pi * np.arange(n_h) / n_h) if hfov >= 360 \
        else np.radians(np.linspace(-hfov / 2, hfov / 2, n_h))
    E, A = np.meshgrid(el, az, indexing='ij')
    D = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3) @ R.T
    best = np.where(D[:, 2] < 0, -O[2] / np.where(D[:, 2] < 0, D[:, 2], -1), np.inf)
    who = np.full(len(D), -1)
    for i, (x, y, c) in enumerate(cones):
        r0, r1, h = CONE_SHAPES['fs']['big' if c == 4 else 'small']
        t = _ray_frustum(O, D, x, y, r0, r1, h)
        m = t < best
        best[m], who[m] = t[m], i
    ok = (who >= 0) & (best <= 100) & (best >= 0.3)
    return np.array([int((who[ok] == i).sum()) for i in range(len(cones))])


def _check(channels, hfov, n_h, h, pitch_deg, cones):
    O = np.array([0.4, 0.0, h])
    R = _pitch(math.radians(pitch_deg))
    C = np.array(cones, float)
    acc = CoverageAccumulator(C, channels, hfov, h_samples=n_h)
    acc.add_scan_raycast(R, O, (1.0, 0.0, 0.0, 0.0))
    assert acc.max_points.tolist() == _brute(channels, hfov, n_h, O, R, C).tolist()
    return acc


def test_vlp16_rings_thin_out_with_range():
    acc = _check(list(range(-15, 16, 2)), 360, 1800, 0.70, 0,
                 [[d, 0, 1] for d in (3, 6, 10, 15, 22, 30)])
    assert acc.max_rings[0] > acc.max_rings[2] >= 1


def test_atx_pitched_and_side_cones():
    atx = list(np.linspace(-10, 10, 256))
    _check(atx, 120, 1500, 0.57, 3, [[d, 0, 1] for d in (3, 6, 10, 15, 22, 30)])
    _check(atx, 120, 1500, 0.57, 0, [[5, 3, 1], [5, -3, 1], [2, 1.5, 2], [15, 8, 1], [3, -6, 1]])


def test_occlusion_and_big_cone():
    acc = _check(list(range(-15, 16, 2)), 360, 1800, 0.70, 0,
                 [[8, 0, 1], [10, 0.05, 1], [9, 3, 4], [12, -20, 2]])
    assert acc.max_points[1] < acc.max_points[0]          # partly hidden behind cone 0


def test_status_classes():
    acc = CoverageAccumulator(np.array([[10, 0, 1], [10.6, 0, 1], [10, 40, 1]]),
                              list(range(-15, 16, 2)), 360, max_range_m=30, h_samples=1800)
    acc.add_scan_raycast(np.eye(3), np.array([0, 0, 0.7]), (1.0, 0.0, 0.0, 0.0))
    st = acc.status()
    assert st[2] == 'never_in_view'          # beyond max range
    assert st[0] in ('detected', 'hit_only')
