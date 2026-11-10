"""
Localisation metric: how far the stack's own pose is from the simulator's true pose.

localization_eval_node.py records, 20 times a second of sim time, three poses of the
car in the map frame:
  truth  the simulator's (TF map -> base_footprint_gt)
  est    the stack's (TF map -> odom -> base_footprint: odometry EKF + cone SLAM)
  odom   dead reckoning alone: the odometry EKF, lined up with the truth at the start
         (what the car would have without the lidar)
plus the SLAM's status per scan and its final cone map.

Every number is taken over the lap by distance, one sample per `STEP_M` metres of
true travel (standing at the start does not count), with no cap and as a full
distribution (min, 5th..95th percentiles, max, mean, RMS):
  position   |est - truth| in metres, and split into along-track (positive = ahead
             of the car) and cross-track (positive = to the car's left)
  heading    est yaw - true yaw in degrees
  final      the same at the end of the lap
  map        SLAM cones paired one-to-one with the true cones (within MATCH_M):
             cones mapped, position error of each mapped cone, map cones with no
             true cone (false: on the track if within ON_TRACK_M of the driven path,
             else off it), true cones mapped twice (duplicates)
  scans      cones detected and paired per lidar scan

Map building (map_building(), from the SLAM's map after every scan): the lap-1 view
the planner has, every STEP_M metres of the lap.  A true cone is usable when a map
cone is paired with it and lies within a tolerance (MAP_TOLS_M, headline MAP_TOL_M) of
where the cone truly is, both taken relative to the car (the map cone from the stack's
own pose, the true cone from the true pose): that is where the planner puts it.
  ahead      the distance along the driven path to the first cone ahead that is not
             usable, or to a false map cone on the track (same rule as lidar_stats'
             known ahead, which asks only whether the lidar's rays reached the cone);
             on a loop it wraps round.  Taken over the lap up to the point where the
             whole map is usable.  No cap.
  entered    per cone, how far the car still had to drive along lap 1 to reach it
             when it first became usable (negative: only once the car had passed it)
  local      the car-relative error of every paired map cone up to LOCAL_WINDOW_M
             ahead, over the lap
True cones closer together than CLOSE_M (the SLAM keeps its cones at least that far
apart: the doubled big orange cones at a timing line) count as one, usable when any of
them is.  Cones further than NEAR_PATH_M from the driven path are not on the lap.
"""

import math
import os

import numpy as np

STEP_M = 0.5
MATCH_M = 1.0
ON_TRACK_M = 5.0
PCTS = (5, 25, 50, 75, 95)
MAP_TOLS_M = (0.1, 0.25, 0.5)
MAP_TOL_M = 0.25
CLOSE_M = 1.0              # falcon_localization ConeSLAM min_separation_m
NEAR_PATH_M = 4.0          # as lidar_stats.path_position
LOCAL_WINDOW_M = 30.0


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def dist(v, digits=3):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if not len(v):
        return None
    out = {f'p{p}': round(float(np.percentile(v, p)), digits) for p in PCTS}
    out.update(min=round(float(v.min()), digits), max=round(float(v.max()), digits),
               mean=round(float(v.mean()), digits),
               rms=round(float(np.sqrt(np.mean(v ** 2))), digits), n=int(len(v)))
    return out


def travelled(truth):
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(truth[:, :2], axis=0).T))])


def by_distance(s, step=STEP_M):
    """Indices of the first sample at each `step` metres of travel."""
    if not len(s):
        return np.zeros(0, int)
    marks = np.arange(0.0, s[-1] + 1e-9, step)
    return np.unique(np.searchsorted(s, marks))


def pose_errors(truth, est):
    """(position, along, cross, heading in deg) of est against truth, per sample."""
    dx, dy = est[:, 0] - truth[:, 0], est[:, 1] - truth[:, 1]
    c, s = np.cos(truth[:, 2]), np.sin(truth[:, 2])
    return (np.hypot(dx, dy), c * dx + s * dy, -s * dx + c * dy,
            np.degrees(wrap(est[:, 2] - truth[:, 2])))


def error_stats(truth, est, idx):
    pos, along, cross, head = pose_errors(truth, est)
    return {
        'position_m': dist(pos[idx]),
        'along_m': dist(along[idx]),
        'cross_m': dist(cross[idx]),
        'cross_abs_m': dist(np.abs(cross[idx])),
        'heading_deg': dist(head[idx]),
        'heading_abs_deg': dist(np.abs(head[idx])),
        'final': {'position_m': round(float(pos[-1]), 3), 'along_m': round(float(along[-1]), 3),
                  'cross_m': round(float(cross[-1]), 3), 'heading_deg': round(float(head[-1]), 3)},
    }


def pair(landmarks, cones, match_m=MATCH_M):
    """One-to-one pairs of map cones with true cones (Hungarian on distance, within
    match_m): (map index, true index, distance), and the unpaired map cones split into
    false ones and second copies of a paired true cone (within match_m of one)."""
    landmarks = np.asarray(landmarks, float).reshape(-1, 2)
    cones = np.asarray(cones, float).reshape(-1, 2)
    none = np.zeros(0, int)
    if not len(landmarks) or not len(cones):
        return none, none, np.zeros(0), np.arange(len(landmarks)), none
    from scipy.optimize import linear_sum_assignment
    d = np.hypot(landmarks[:, None, 0] - cones[None, :, 0], landmarks[:, None, 1] - cones[None, :, 1])
    reach = d < match_m
    nl, nc = reach.sum(axis=1), reach.sum(axis=0)
    # A map cone and a true cone that only reach each other pair up whatever the rest
    # does; the Hungarian runs on the others.
    one = np.flatnonzero(nl == 1)
    j1 = reach[one].argmax(axis=1)
    keep = nc[j1] == 1
    rows, cols = one[keep], j1[keep]
    li = np.setdiff1d(np.flatnonzero(nl > 0), rows)
    ci = np.setdiff1d(np.flatnonzero(nc > 0), cols)
    if len(li) and len(ci):
        sub = d[np.ix_(li, ci)]
        r, c = linear_sum_assignment(np.where(sub < match_m, sub, 1e6))
        ok = sub[r, c] < match_m
        rows, cols = np.concatenate([rows, li[r[ok]]]), np.concatenate([cols, ci[c[ok]]])
    unpaired = np.setdiff1d(np.arange(len(landmarks)), rows)
    # An unpaired map cone near a true cone that is already paired: the same cone twice.
    near = d[unpaired].min(axis=1) < match_m if len(unpaired) else np.zeros(0, bool)
    return rows, cols, d[rows, cols], unpaired[~near], unpaired[near]


def map_quality(landmarks, cones, match_m=MATCH_M, path=None):
    """One-to-one pairing of SLAM cones with true cones (Hungarian on distance).
    path (N, 2): the true driven path, to tell false cones on the track from off it."""
    landmarks = np.asarray(landmarks, float).reshape(-1, 2)
    cones = np.asarray(cones, float).reshape(-1, 2)
    out = {'true_cones': int(len(cones)), 'map_cones': int(len(landmarks))}
    if not len(landmarks) or not len(cones):
        out.update(mapped=0, mapped_fraction=0.0, false=int(len(landmarks)), duplicates=0,
                   false_on_track=None, false_off_track=None, error_m=None)
        return out
    rows, _, err, false, twice = pair(landmarks, cones, match_m)
    on = None
    if path is not None and len(false):
        path = np.asarray(path, float).reshape(-1, 2)[::5]
        dp = np.hypot(landmarks[false, None, 0] - path[None, :, 0],
                      landmarks[false, None, 1] - path[None, :, 1]).min(axis=1)
        on = int((dp <= ON_TRACK_M).sum())
    elif path is not None:
        on = 0
    out.update(mapped=int(len(rows)), mapped_fraction=round(float(len(rows)) / len(cones), 4),
               false=int(len(false)), duplicates=int(len(twice)),
               false_on_track=on, false_off_track=None if on is None else int(len(false)) - on,
               error_m=dist(err))
    return out


def to_car(xy, pose):
    """Map-frame points (N, 2) in the frame of a car at pose (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = xy[:, 0] - pose[0], xy[:, 1] - pose[1]
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def path_position(points, path, s, near_m=NEAR_PATH_M):
    """Distance along the driven path (N, 2), travelled s (N,), at which the car passes
    each point (nan: it is never within near_m of the path)."""
    points = np.asarray(points, float).reshape(-1, 2)
    if not len(points) or not len(path):
        return np.full(len(points), np.nan)
    d = np.hypot(points[:, None, 0] - path[None, :, 0], points[:, None, 1] - path[None, :, 1])
    j = d.argmin(axis=1)
    out = np.asarray(s, float)[j]
    out[d[np.arange(len(points)), j] > near_m] = np.nan
    return out


def lap_shape(truth, s):
    """(lap length, closed?): a lap that ends within 3 m of where it started is a loop."""
    gap = float(np.hypot(*(truth[-1, :2] - truth[0, :2])))
    closed = gap < 3.0 and s[-1] > 50.0
    return float(s[-1]) + (gap if closed else 0.0), closed


def cone_groups(cones, close_m=CLOSE_M):
    """A group number per cone (0..G-1): cones closer than close_m share one."""
    g = np.arange(len(cones))
    d = np.hypot(cones[:, None, 0] - cones[None, :, 0], cones[:, None, 1] - cones[None, :, 1])
    for a, b in zip(*np.nonzero(np.triu(d < close_m, 1))):
        g[g == g[b]] = g[a]
    return np.unique(g, return_inverse=True)[1].reshape(-1)


def map_building(t, truth, est, map_t, map_n, map_xy, cones, tols=MAP_TOLS_M, step=STEP_M,
                 samples=False):
    """Lap-1 map statistics from the SLAM's map after each scan (see the module doc).
    map_t (K,) stamps, map_n (K,) map size, map_xy (sum map_n, 2) the maps in turn.
    samples: also return the values behind the distributions (`_samples`, arrays)."""
    t = np.asarray(t, float)
    truth, est = (np.asarray(a, float).reshape(-1, 3) for a in (truth, est))
    cones = np.asarray(cones, float)
    cones = cones.reshape(len(cones), -1)[:, :2]
    map_t, map_n = np.asarray(map_t, float), np.asarray(map_n, int)
    map_xy = np.asarray(map_xy, float).reshape(-1, 2)
    s = travelled(truth)
    L, closed = lap_shape(truth, s)
    path, ps = truth[::2, :2], s[::2]
    grp = cone_groups(cones)
    G = int(grp.max()) + 1 if len(grp) else 0
    s_cone = path_position(cones, path, ps)
    s_grp = np.full(G, np.inf)
    np.fmin.at(s_grp, grp, np.where(np.isfinite(s_cone), s_cone, np.inf))
    on = np.isfinite(s_grp)
    s_grp[~on] = np.nan
    end = float(np.nanmax(s_grp)) if on.any() else L
    off = np.concatenate([[0], np.cumsum(map_n)]).astype(int)
    idx = by_distance(s, step)
    ks = np.searchsorted(map_t, t[idx], side='right') - 1

    ahead = {tol: np.empty(len(idx)) for tol in tols}
    whole = {tol: np.zeros(len(idx), bool) for tol in tols}
    entered = np.full(G, np.nan)
    local = []
    last_k = None
    for j, (i, k) in enumerate(zip(idx, ks)):
        if k != last_k:
            lms = map_xy[off[k]:off[k + 1]] if k >= 0 else np.zeros((0, 2))
            rows, cols, _, false, _ = pair(lms, cones)
            s_false = path_position(lms[false], path, ps, ON_TRACK_M)
            s_false = s_false[np.isfinite(s_false)]
            last_k = k
        e = np.full(len(cones), np.inf)
        if len(rows):
            e[cols] = np.hypot(*(to_car(lms[rows], est[i]) - to_car(cones[cols], truth[i])).T)
        eg = np.full(G, np.inf)
        np.fmin.at(eg, grp, e)
        rel = s_grp - s[i]
        fwd = rel % L if closed else rel
        local.append(eg[on & (fwd > 1e-6) & (fwd <= LOCAL_WINDOW_M) & np.isfinite(eg)])
        fs = (s_false - s[i]) % L if closed else s_false - s[i]
        fs = fs[fs > 1e-6]
        for tol in tols:
            d = fwd[on & ~(eg <= tol)]
            d = np.concatenate([d[d > 1e-6], fs])
            if len(d):
                ahead[tol][j] = d.min()
            else:
                whole[tol][j] = True
                ahead[tol][j] = L if closed else max(0.0, end - s[i])
        # Lap 1: how far the car still had to drive to reach the cone (not wrapped
        # round the loop: a cone just behind the start is reached at the lap's end).
        new = on & np.isnan(entered) & (eg <= MAP_TOL_M)
        entered[new] = rel[new]

    out = {'tol_m': MAP_TOL_M, 'tols_m': list(tols), 'cones': int(on.sum()),
           'cones_true': int(len(cones)), 'scans': int(len(map_t)), 'lap_m': round(L, 1),
           'ahead_m': {}, 'complete_at_m': {}}
    for tol in tols:
        w = whole[tol]
        # Once the whole map is usable, "ahead" is the whole lap for any lidar, so the
        # distribution covers the lap up to there (as lidar_stats' known ahead).
        before = ahead[tol][~w] if (~w).any() else ahead[tol]
        out['ahead_m'][str(tol)] = dist(before, 1)
        out['complete_at_m'][str(tol)] = round(float(s[idx][np.argmax(w)]), 1) if w.any() else None
    w = whole[MAP_TOL_M] if MAP_TOL_M in whole else np.zeros(len(idx), bool)
    before = ahead[MAP_TOL_M][~w] if MAP_TOL_M in ahead and (~w).any() else np.zeros(0)
    out['ahead_at_least'] = {str(d): round(float((before >= d).mean()), 3) if len(before) else None
                             for d in (10, 20, 30, 50, 75, 100)}
    out['entered_m'] = dist(entered[on & np.isfinite(entered)], 1)
    out['never'] = int((on & np.isnan(entered)).sum())
    out['local_error_m'] = dist(np.concatenate(local) if local else [], 3)
    if samples:
        out['_samples'] = {'ahead_m': before, 'entered_m': entered[on & np.isfinite(entered)],
                           'local_error_m': np.concatenate(local) if local else np.zeros(0)}
    if MAP_TOL_M in ahead:
        k = max(1, int(round(1.0 / step)))
        out['trace'] = {'s_m': [round(float(v), 1) for v in s[idx][::k]],
                        'ahead_m': [round(float(v), 1) for v in ahead[MAP_TOL_M][::k]]}
    return out


def summarize(t, truth, est, odom, status=None, landmarks=None, cones=None, maps=None,
              samples=False):
    """Summary dict for one run (arrays: t (N,), poses (N, 3) x, y, yaw; maps: the
    SLAM's map after each scan as (map_t, map_n, map_xy), see map_building)."""
    truth, est, odom = (np.asarray(a, float).reshape(-1, 3) for a in (truth, est, odom))
    s = travelled(truth)
    idx = by_distance(s)
    out = {'samples': int(len(t)), 'distance_m': round(float(s[-1]) if len(s) else 0.0, 1),
           'duration_s': round(float(t[-1] - t[0]) if len(t) else 0.0, 1),
           'step_m': STEP_M,
           'slam': error_stats(truth, est, idx), 'dead_reckoning': error_stats(truth, odom, idx)}
    if status is not None and len(status):
        st = np.asarray(status, float)
        out['scans'] = {'count': int(len(st)), 'detections': dist(st[:, 1], 1),
                        'matched': dist(st[:, 2], 1),
                        'map_cones_final': int(st[-1, 4]),
                        'ms_per_scan': dist(st[:, 5], 2)}
    if landmarks is not None and cones is not None:
        out['map'] = map_quality(landmarks, np.asarray(cones)[:, :2], path=truth[:, :2])
    if maps is not None and cones is not None and len(maps[0]) and len(cones):
        out['map_building'] = map_building(t, truth, est, *maps, cones, samples=samples)
    return out


def trace(t, truth, est, odom, step=1.0):
    """Errors along the lap every `step` metres, for plots."""
    truth, est, odom = (np.asarray(a, float).reshape(-1, 3) for a in (truth, est, odom))
    s = travelled(truth)
    idx = by_distance(s, step)
    pe, _, ce, he = pose_errors(truth[idx], est[idx])
    po, _, co, ho = pose_errors(truth[idx], odom[idx])
    r = lambda v: [round(float(x), 3) for x in v]  # noqa: E731
    return {'s_m': r(s[idx]), 'slam_position_m': r(pe), 'slam_cross_m': r(ce),
            'slam_heading_deg': r(he), 'dr_position_m': r(po), 'dr_cross_m': r(co),
            'dr_heading_deg': r(ho), 'x': r(truth[idx, 0]), 'y': r(truth[idx, 1])}


def load(run_dir, samples=False):
    """summary + trace of a run folder's localization.npz (None if there is none).
    samples: keep map_building's per-sample values (`_samples`, for pooling laps)."""
    path = os.path.join(run_dir, 'localization.npz')
    if not os.path.isfile(path):
        return None
    z = np.load(path)
    t, truth, est, odom = z['t'], z['truth'], z['est'], z['odom']
    out = summarize(t, truth, est, odom, z['status'] if 'status' in z else None,
                    z['landmarks'] if 'landmarks' in z else None,
                    z['cones'] if 'cones' in z else None,
                    (z['map_t'], z['map_n'], z['map_xy']) if 'map_t' in z else None, samples)
    out['complete'] = bool(z['complete']) if 'complete' in z else None
    out['trace'] = trace(t, truth, est, odom)
    return out


def _get(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or d.get(k) is None:
            return None
        d = d[k]
    return d


def laps(run_dirs):
    """The headline numbers of each lap (one run folder per lap) and their spread."""
    rows, pooled, map_trace = [], {}, None
    for d in run_dirs:
        s = load(d, samples=True)
        if s is None or not s.get('complete', True):
            continue
        if s.get('map_building'):
            for k, v in s['map_building'].pop('_samples').items():
                pooled.setdefault(k, []).append(v)
            if map_trace is None:
                map_trace = dict(s['map_building']['trace'],
                                 run=os.path.basename(os.path.normpath(d)),
                                 complete_at_m=s['map_building']['complete_at_m'][str(MAP_TOL_M)])
        e = s['slam']
        m = s.get('map') or {}
        mb = s.get('map_building') or {}
        rows.append({'run': os.path.basename(os.path.normpath(d)),
                     'p50': e['position_m']['p50'], 'p95': e['position_m']['p95'],
                     'max': e['position_m']['max'], 'final': e['final']['position_m'],
                     'heading_p95': e['heading_abs_deg']['p95'],
                     'map_error_p95': _get(m, 'error_m', 'p95'),
                     'map_error_max': _get(m, 'error_m', 'max'),
                     'false_on_track': m.get('false_on_track'),
                     'false_off_track': m.get('false_off_track'),
                     'duplicates': m.get('duplicates'),
                     'mapped': m.get('mapped'),
                     'ahead_p5': _get(mb, 'ahead_m', str(MAP_TOL_M), 'p5'),
                     'ahead_p50': _get(mb, 'ahead_m', str(MAP_TOL_M), 'p50'),
                     'ahead_tight_p50': _get(mb, 'ahead_m', str(MAP_TOLS_M[0]), 'p50'),
                     'ahead_loose_p50': _get(mb, 'ahead_m', str(MAP_TOLS_M[-1]), 'p50'),
                     'entered_p5': _get(mb, 'entered_m', 'p5'),
                     'entered_p50': _get(mb, 'entered_m', 'p50'),
                     'local_error_p95': _get(mb, 'local_error_m', 'p95'),
                     'dr_max': s['dead_reckoning']['position_m']['max']})
    out = {'n': len(rows), 'laps': rows,
           'n_map': sum(r['ahead_p50'] is not None for r in rows)}
    if pooled:
        # Every map lap's samples together: each lap counts by its length.
        out['map_pooled'] = {k: dist(np.concatenate(v), 3 if k == 'local_error_m' else 1)
                             for k, v in pooled.items()}
        out['map_trace'] = map_trace
    for k in ('p50', 'p95', 'max', 'final', 'heading_p95', 'map_error_p95', 'map_error_max',
              'false_on_track', 'false_off_track', 'duplicates', 'mapped', 'ahead_p5',
              'ahead_p50', 'ahead_tight_p50', 'ahead_loose_p50', 'entered_p5', 'entered_p50',
              'local_error_p95', 'dr_max'):
        v = [r[k] for r in rows if r[k] is not None]
        if v:
            out[k] = {'min': round(min(v), 3), 'median': round(float(np.median(v)), 3),
                      'max': round(max(v), 3)}
    return out


def summary_line(s):
    e, d = s['slam'], s['dead_reckoning']
    m = s.get('map') or {}
    return (f"position p50/p95/max {e['position_m']['p50']:.2f}/{e['position_m']['p95']:.2f}/"
            f"{e['position_m']['max']:.2f} m, heading p95 {e['heading_abs_deg']['p95']:.2f} deg, "
            f"final {e['final']['position_m']:.2f} m (dead reckoning final "
            f"{d['final']['position_m']:.2f} m, max {d['position_m']['max']:.2f} m); "
            f"map {m.get('mapped', 0)}/{m.get('true_cones', 0)} cones, "
            f"{m.get('false_on_track')} false on the track, {m.get('false_off_track')} off it, "
            f"{m.get('duplicates', 0)} twice"
            + (f", error p95 {m['error_m']['p95']:.2f} m" if m.get('error_m') else '')
            + _map_building_line(s.get('map_building')))


def _map_building_line(mb):
    a = _get(mb, 'ahead_m', str(MAP_TOL_M))
    if not a:
        return ''
    e = mb.get('entered_m') or {}
    le = mb.get('local_error_m') or {}
    return (f"; map ahead (cones within {MAP_TOL_M} m) p5/p50 {a['p5']:.1f}/{a['p50']:.1f} m, "
            f"cones enter it p5/p50 {e.get('p5', float('nan')):.1f}/{e.get('p50', float('nan')):.1f}"
            f" m ahead, {mb.get('never')} never, local error p95 "
            f"{le.get('p95', float('nan')):.3f} m")
