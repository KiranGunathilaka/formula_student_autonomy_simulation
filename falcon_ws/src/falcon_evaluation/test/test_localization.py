"""Localisation metric (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_evaluation import localization as loc


def _lap(n=2000, radius=20.0):
    th = np.linspace(0, 2 * np.pi, n)
    truth = np.column_stack([radius * np.sin(th), radius * (1 - np.cos(th)), th])
    return np.linspace(0, 60, n), truth


def test_offset_estimate_splits_into_along_and_cross_track():
    t, truth = _lap()
    est = truth.copy()
    # 0.3 m ahead and 0.1 m to the left of the true car, 1 deg off.
    c, s = np.cos(truth[:, 2]), np.sin(truth[:, 2])
    est[:, 0] += 0.3 * c - 0.1 * s
    est[:, 1] += 0.3 * s + 0.1 * c
    est[:, 2] += math.radians(1.0)
    out = loc.summarize(t, truth, est, truth)
    e = out['slam']
    assert abs(e['along_m']['p50'] - 0.3) < 1e-3 and abs(e['cross_m']['p50'] - 0.1) < 1e-3
    assert abs(e['position_m']['max'] - math.hypot(0.3, 0.1)) < 1e-3
    assert abs(e['heading_deg']['mean'] - 1.0) < 1e-3
    assert out['dead_reckoning']['position_m']['max'] == 0.0


def test_standing_still_does_not_count():
    t, truth = _lap()
    still = np.repeat(truth[:1], 5000, axis=0)
    truth2 = np.vstack([still, truth])
    est = truth2.copy()
    est[:5000, 0] += 5.0                    # wrong only while standing
    out = loc.summarize(np.arange(len(truth2)) * 0.05, truth2, est, truth2)
    assert out['slam']['position_m']['p95'] < 1e-6


def test_map_pairs_cones_once_and_counts_false_and_duplicates():
    cones = np.array([[0, 0], [5, 0], [10, 0], [15, 0]], float)
    lms = np.array([[0.1, 0], [5, 0.2], [5.3, 0.1], [30, 0]], float)
    path = np.column_stack([np.linspace(0, 15, 50), np.full(50, 1.0)])
    m = loc.map_quality(lms, cones, path=path)
    assert m['mapped'] == 2 and m['duplicates'] == 1 and m['false'] == 1
    assert m['false_on_track'] == 0 and m['false_off_track'] == 1          # 15 m from the path
    assert abs(m['error_m']['max'] - 0.2) < 1e-3


def test_laps_spread_over_run_folders(tmp_path):
    t, truth = _lap()
    for i, off in enumerate((0.1, 0.3)):
        d = tmp_path / f'lidar_s{i + 1}'
        d.mkdir()
        est = truth.copy()
        est[:, 1] += off
        np.savez(d / 'localization.npz', t=t, truth=truth, est=est, odom=truth, complete=True)
    out = loc.laps([str(tmp_path / 'lidar_s1'), str(tmp_path / 'lidar_s2'), str(tmp_path / 'none')])
    assert out['n'] == 2
    assert abs(out['p95']['min'] - 0.1) < 1e-3 and abs(out['p95']['max'] - 0.3) < 1e-3


def _mapping_lap(view_m=15.0, offset=(0.0, 0.0), extra=None, n=2400, radius=20.0):
    """A round lap with cones 2 m either side every 10 deg (3.5 m apart on the path),
    and a map that holds every cone that has been within view_m ahead of the car."""
    t = np.linspace(0, 120, n)
    th = np.linspace(0, 2 * np.pi, n)
    truth = np.column_stack([radius * np.sin(th), radius * (1 - np.cos(th)), th])
    a = np.radians(np.arange(5, 360, 10))
    cones = np.vstack([np.column_stack([r * np.sin(a), radius - r * np.cos(a)])
                       for r in (radius - 2, radius + 2)])
    cones = np.column_stack([cones, np.ones(len(cones))])
    seen = np.zeros(len(cones), bool)
    map_t, map_n, map_xy = [], [], []
    for i in range(0, n, 4):                     # 10 Hz
        p = loc.to_car(cones[:, :2], truth[i])
        seen |= (p[:, 0] > 0) & (np.hypot(p[:, 0], p[:, 1]) < view_m)
        xy = cones[seen, :2] + np.asarray(offset)
        if extra is not None:
            xy = np.vstack([xy, extra])
        map_t.append(t[i])
        map_n.append(len(xy))
        map_xy.append(xy)
    return t, truth, cones, np.array(map_t), np.array(map_n), np.vstack(map_xy)


def test_map_ahead_follows_what_the_map_holds():
    t, truth, cones, mt, mn, mxy = _mapping_lap(view_m=15.0)
    out = loc.map_building(t, truth, truth, mt, mn, mxy, cones)
    a = out['ahead_m']['0.25']
    # The first missing cone lies 15 m away in a straight line, up to one cone gap
    # (3.5 m) further along the path.
    assert 12.0 < a['p50'] < 19.0 and a['p5'] > 10.0
    assert out['never'] == 0 and out['entered_m']['p50'] > 12.0
    assert out['local_error_m']['max'] < 1e-6
    near = loc.map_building(t, truth, truth, *_mapping_lap(view_m=6.0)[3:], cones)
    assert near['ahead_m']['0.25']['p50'] < a['p50'] - 6.0


def test_map_ahead_needs_cones_where_they_are():
    t, truth, cones, mt, mn, mxy = _mapping_lap(offset=(0.3, 0.0))
    out = loc.map_building(t, truth, truth, mt, mn, mxy, cones)
    # 0.3 m off: no cone is usable at 0.1 or 0.25 m, every one is at 0.5 m.
    assert out['ahead_m']['0.25']['p95'] < 4.0 and out['never'] == len(cones)
    assert out['ahead_m']['0.5']['p50'] > 12.0
    # The same map seen from a pose that is 0.3 m off the same way is right where the
    # planner needs it: only errors relative to the car count.
    est = truth.copy()
    est[:, 0] += 0.3
    ok = loc.map_building(t, truth, est, mt, mn, mxy, cones)
    assert ok['ahead_m']['0.25']['p50'] > 12.0 and ok['local_error_m']['max'] < 1e-6


def test_false_cone_on_the_track_ends_the_map_ahead():
    t, truth, cones, mt, mn, mxy = _mapping_lap(view_m=15.0)
    clean = loc.map_building(t, truth, truth, mt, mn, mxy, cones)
    ghost = np.array([[20.0, 20.0]])              # on the driven path, half a lap round
    t, truth, cones, mt, mn, mxy = _mapping_lap(view_m=15.0, extra=ghost)
    out = loc.map_building(t, truth, truth, mt, mn, mxy, cones)
    assert out['ahead_m']['0.25']['min'] < 1.0 < clean['ahead_m']['0.25']['min']


def test_cones_closer_than_the_slam_spacing_count_once():
    t, truth, cones, mt, mn, mxy = _mapping_lap(view_m=15.0)
    twin = cones[:1].copy()
    twin[0, 0] += 0.6                             # a doubled cone the map holds once
    both = np.vstack([cones, twin])
    a = loc.map_building(t, truth, truth, mt, mn, mxy, both)
    assert a['cones'] == len(cones) and a['never'] == 0
    assert abs(a['ahead_m']['0.25']['p50']
               - loc.map_building(t, truth, truth, mt, mn, mxy, cones)['ahead_m']['0.25']['p50']) < 0.2


def test_laps_pool_the_map_laps(tmp_path):
    t, truth, cones, mt, mn, mxy = _mapping_lap(view_m=15.0)
    d = tmp_path / 'lidar_s1'
    d.mkdir()
    np.savez(d / 'localization.npz', t=t, truth=truth, est=truth, odom=truth, complete=True)
    for i, view in ((2, 15.0), (3, 6.0)):
        d = tmp_path / f'lidar_s{i}'
        d.mkdir()
        _, _, _, mt, mn, mxy = _mapping_lap(view_m=view)
        np.savez(d / 'localization.npz', t=t, truth=truth, est=truth, odom=truth, complete=True,
                 landmarks=mxy[-mn[-1]:], cones=cones, map_t=mt, map_n=mn, map_xy=mxy)
    out = loc.laps([str(tmp_path / f'lidar_s{i}') for i in (1, 2, 3)])
    assert out['n'] == 3 and out['n_map'] == 2
    assert out['ahead_p50']['max'] - out['ahead_p50']['min'] > 6.0
    assert out['map_pooled']['ahead_m']['n'] > 300 and out['map_trace']['run'] == 'lidar_s2'
    assert '_samples' not in str(out)
