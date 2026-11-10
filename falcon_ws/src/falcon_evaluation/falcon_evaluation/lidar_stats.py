"""
lidar_stats: what each lidar sees of a track, lap by lap, as statistics.

From Gazebo runs (the simulated lidar's own point clouds, recorded by cone_coverage
as frames.npz):

  ros2 run falcon_evaluation lidar_stats --runs ~/falcon_runs/lidar_gazebo/* \
      --out ~/falcon_runs/lidar_gazebo

or from an offline drive (no Gazebo: a kinematic ADS-DV and the profile's exact rays):

  ros2 run falcon_evaluation lidar_stats --tracks lidar_test_track \
      --lidars hesai_atx velodyne_vlp16@0.70 ouster_os1_64@0.70

Offline, the car drives one lap with the same planner and pure pursuit as the sim
(falcon_planning), steering on the cones a simulated perception would see (20 m,
180 deg), so every lidar is judged on the same drive.  Ten times a second each
lidar's beam pattern (its profile in falcon_description/config/lidars) is cast from
its mount against rulebook-size cones, the ground and the car's bodywork
(falcon_evaluation/coverage.py).  Lidars: a profile name, `name@z` to mount it z
metres up (default: the vehicle's lidar mount), `a+b` for two lidars together.
Tracks: an eufs_tracks name, a track CSV path, or a name in $FALCON_RUNS_DIR/tracks
or falcon_bringup/tracks.

Every frame keeps the points and the channels on every cone, so any detection rule
`points:rings` can be applied afterwards (--rules, default 3:1 10:2 20:3).  For each:
  detected        cones that met the rule in at least one frame
  first detection the range at which each cone first met the rule (10th, 50th, 90th
                  percentile and the furthest)
  known ahead     lap-1 view, every 0.5 m of the lap: how far along the driven path
                  every cone is already in the map; the first cone ahead not yet
                  detected ends it.  On a loop it wraps round.  No cap.  Kept as
                  its full distribution (min, 1st..95th percentile, max, mean) and
                  the share of the lap at or above 10..200 m, both over the lap up
                  to the point where every cone is in the map (map_complete_at_m;
                  from there on it is the whole lap for any lidar), and a trace
  cones per frame cones meeting the rule at each point of the lap (5th, 50th, 95th
                  percentile), and metres driven with fewer than 2
  points vs range mean points and channels on a cone in view, by range band
A Gazebo run that also has localization.npz (sim.launch.py localization:=slam) gets
its localisation errors too (localization.py: position, along/cross-track, heading,
map, map building, against dead reckoning alone), under `localization`; with
--repeats the spread over repeat laps (`localization.laps`), and with --variant the
same for a variant of the stack (`localization.variants`).
Writes <out>/lidar_stats.json (and prints a summary).
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

from falcon_evaluation import localization
from falcon_evaluation.coverage import RANGE_BINS, CoverageAccumulator

COLOURS = {'blue': 1, 'yellow': 2, 'orange': 3, 'big_orange': 4}
POINT_SWEEP = (1, 2, 3, 5, 7, 10, 15, 20, 30, 40, 60)
PCTS = (1, 5, 10, 25, 50, 75, 90, 95)        # percentiles kept of every distribution
THRESHOLDS_M = (10, 20, 30, 50, 75, 100, 150, 200)
GRID_M = 0.5                                  # known ahead: one sample per 0.5 m of lap
TRACE_M = 1.0                                 # ... and the trace along the lap per 1 m
SIM_VIEW_M, SIM_VIEW_DEG = 20.0, 180.0       # sim.launch.py's simulated /cones
FRAME_DT, CTRL_DT = 0.1, 0.05


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def track_csv(track):
    """Path of the track CSV for a track:= value (see sim.launch.py)."""
    if track.endswith('.csv') and os.path.isfile(os.path.expanduser(track)):
        return os.path.expanduser(track)
    name = os.path.basename(track if track.endswith('.csv') else f'{track}.csv')
    runs = os.path.join(os.path.expanduser(os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')),
                        'tracks', name)
    if os.path.isfile(runs):
        return runs
    from ament_index_python.packages import get_package_share_directory
    own = os.path.join(get_package_share_directory('falcon_bringup'), 'tracks', name)
    if os.path.isfile(own):
        return own
    path = os.path.join(get_package_share_directory('eufs_tracks'), 'csv', f'{track}.csv')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'track {track}: no {runs} and no eufs_tracks {path}')
    return path


def load_track(track):
    cones, start = [], (0.0, 0.0, 0.0)
    with open(track_csv(track)) as f:
        for row in csv.DictReader(f):
            if row['tag'] == 'car_start':
                start = (float(row['x']), float(row['y']), float(row['direction']))
            elif row['tag'] in COLOURS:
                cones.append((float(row['x']), float(row['y']), COLOURS[row['tag']]))
    return np.array(cones, dtype=float), start


def to_body(pts, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(pts, dtype=float).reshape(-1, 2) - (x, y)
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def to_map(pts, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(pts, dtype=float).reshape(-1, 2)
    return np.stack([c * p[:, 0] - s * p[:, 1] + x, s * p[:, 0] + c * p[:, 1] + y], axis=1)


def drive(cones, start, vehicle, speed, max_dist):
    """One lap with falcon_planning on a kinematic car: poses every FRAME_DT."""
    from falcon_planning import centerline, pursuit
    wb, max_steer = float(vehicle['wheelbase_m']), float(vehicle['max_steering_rad'])
    x, y, yaw = start
    v, dist, t = 0.0, 0.0, 0.0
    seen = np.zeros(len(cones), dtype=bool)
    prev_map, frames, still = None, [], 0.0
    next_frame = 0.0
    while dist < max_dist:
        b = to_body(cones[:, :2], x, y, yaw)
        seen |= (np.hypot(b[:, 0], b[:, 1]) < SIM_VIEW_M) & \
            (np.abs(np.arctan2(b[:, 1], b[:, 0])) <= math.radians(SIM_VIEW_DEG) / 2)
        col = cones[seen, 2]
        bs = b[seen]
        prev_body = to_body(prev_map, x, y, yaw) if prev_map is not None else None
        path, _, _ = centerline.plan_centerline(bs[col == 1], bs[col == 2], None,
                                                orange=bs[col >= 3], prev_path=prev_body)
        if len(path) >= 2:
            prev_map = to_map(path, x, y, yaw)
        pb = to_body(prev_map, x, y, yaw) if prev_map is not None else None
        lh = pursuit.lookahead_point(pb, max(2.5, 0.6 * v)) if pb is not None else None
        if lh is None:
            st, sp = 0.0, 0.0
        else:
            st = max(-max_steer, min(max_steer, pursuit.steering_angle(lh, wb, -wb / 2)))
            sp = pursuit.speed_for_path(pb, speed, 1.0, 3.0, 8.0, centerline.path_curvature)
        v += max(-4.0 * CTRL_DT, min(3.0 * CTRL_DT, sp - v))
        if t >= next_frame:
            frames.append((t, x, y, yaw, dist))
            next_frame += FRAME_DT
        x += v * math.cos(yaw) * CTRL_DT
        y += v * math.sin(yaw) * CTRL_DT
        yaw += v * math.tan(st) / wb * CTRL_DT
        dist += v * CTRL_DT
        t += CTRL_DT
        still = still + CTRL_DT if v < 0.05 else 0.0
        if t > 5.0 and still > 1.0:          # end of an open track: no path left
            break
        if dist > 20.0 and math.hypot(x - start[0], y - start[1]) < 1.5:
            frames.append((t, x, y, yaw, dist))
            break                            # closed loop: back at the start
    return np.array(frames)


def parse_lidar(spec, vehicle):
    from falcon_description import config as cfg
    parts = []
    for one in spec.split('+'):
        name, _, z = one.partition('@')
        L = cfg.load_lidar(name)
        mount = dict(vehicle['sensors']['lidar'])
        if z:
            mount['z'] = float(z)
        parts.append((L, mount))
    return parts


def cast_all(cones, frames, parts, vehicle, cone_shape):
    """Points and channels on every cone in every frame, and its range from the lidar."""
    occ = vehicle.get('occluder_box')
    occ = [occ['x_min'], occ['x_max'], occ['y_half'], occ['z_max']] if occ else None
    n_f, n_c = len(frames), len(cones)
    npts = np.zeros((n_f, n_c), dtype=np.int32)
    nring = np.zeros((n_f, n_c), dtype=np.int32)
    rng = np.full((n_f, n_c), np.inf)
    for L, mount in parts:
        h = L['horizontal']
        acc = CoverageAccumulator(cones, L['channels_deg'], float(h['fov_deg']),
                                  float(h.get('centre_deg', 0.0)),
                                  max_range_m=float(L['range']['max_m']),
                                  min_range_m=float(L['range']['min_m']),
                                  h_samples=int(h['samples']), cone_shape=cone_shape,
                                  occluder_box=occ)
        R = rpy_to_R(*(math.radians(mount[k]) for k in ('roll_deg', 'pitch_deg', 'yaw_deg')))
        t_sb = np.array([mount['x'], mount['y'], mount['z']])
        for i, (_, x, y, yaw, _) in enumerate(frames):
            c, s = math.cos(yaw), math.sin(yaw)
            # map -> base_footprint, as (cos, sin, tx, ty) of a rotation by -yaw
            m2b = (c, -s, -(c * x + s * y), s * x - c * y)
            cx, cy, r, view, bs, ts = acc._geometry(R, t_sb, m2b)
            p, k = acc._raycast(R, t_sb, cx, cy, view, bs, ts)
            npts[i] += p
            nring[i] += k
            rng[i] = np.fmin(rng[i], np.where(view, r, np.inf))
    return npts, nring, rng


def path_position(cones, frames, near_m=4.0):
    """Distance along the driven path at which the car passes each cone (nan: never)."""
    d = np.hypot(cones[:, None, 0] - frames[None, :, 1], cones[:, None, 1] - frames[None, :, 2])
    j = np.argmin(d, axis=1)
    s = frames[j, 4].copy()
    s[d[np.arange(len(cones)), j] > near_m] = np.nan
    return s


def lap_of(frames):
    """(lap length, closed?): a lap that ends within 3 m of where it started is a loop."""
    gap = float(np.hypot(frames[-1, 1] - frames[0, 1], frames[-1, 2] - frames[0, 2]))
    closed = gap < 3.0 and frames[-1, 4] > 50.0
    return float(frames[-1, 4]) + (gap if closed else 0.0), closed


def pct(v, ps=PCTS):
    v = np.asarray(v, dtype=float)
    if not len(v):
        return None
    out = {f'p{p}': round(float(np.percentile(v, p)), 1) for p in ps}
    out.update(min=round(float(v.min()), 1), max=round(float(v.max()), 1),
               mean=round(float(v.mean()), 1))
    return out


def known_ahead(det, frames, s_cone):
    """Lap-1 view of the track, every GRID_M metres of the lap.

    At each point of the lap: the distance ahead along the driven path to the first
    cone that has not met the detection rule in any frame so far.  On a loop the
    distance wraps round the lap, so when every cone is in the map it is the whole lap.
    No cap: the only limits are the lap itself and the lidar's datasheet range.
    Returns (grid, distance ahead, everything-known flags, frame index per grid point).
    """
    L, closed = lap_of(frames)
    known = np.maximum.accumulate(det, axis=0)
    ok = np.isfinite(s_cone)
    sc, kn = s_cone[ok], known[:, ok]
    end = float(np.nanmax(sc)) if len(sc) else L
    grid = np.arange(0.0, float(frames[-1, 4]), GRID_M)
    idx = np.clip(np.searchsorted(frames[:, 4], grid, side='right') - 1, 0, len(frames) - 1)
    ahead = np.empty(len(grid))
    whole = np.zeros(len(grid), dtype=bool)
    for j, (s, i) in enumerate(zip(grid, idx)):
        d = sc[~kn[i]] - s
        # A cone the car is level with (d = 0) has been reached, not lying ahead.
        d = d % L if closed else d
        d = d[d > 1e-6]
        if len(d):
            ahead[j] = d.min()
        else:
            whole[j] = True
            ahead[j] = L if closed else max(0.0, end - s)
    return grid, ahead, whole, idx


def stats_for(det, rng, frames, s_cone, trace=False):
    """Statistics of one detection mask (frames x cones)."""
    ever = det.any(axis=0)
    with np.errstate(invalid='ignore'):
        first = np.where(det, rng, -np.inf).max(axis=0)
    first = first[ever & np.isfinite(first)]
    grid, ahead, whole, idx = known_ahead(det, frames, s_cone)
    per_m = det.sum(axis=1)[idx]                    # cones meeting the rule, per metre
    # Once every cone of the lap is in the map, "known ahead" is the whole lap for any
    # lidar (near the end of lap 1 the cones left ahead were seen at the start), so the
    # distribution covers the lap up to that point, and the point is reported apart.
    before = ahead[~whole] if (~whole).any() else ahead
    out = {
        'detected': int(ever.sum()),
        'first_detect_m': pct(first, (10, 50, 90)),
        'known_ahead_m': pct(before),
        'map_complete_at_m': round(float(grid[np.argmax(whole)]), 1) if whole.any() else None,
        'whole_track_known_fraction': round(float(whole.mean()), 3),
        'known_ahead_at_least': {str(d): round(float((before >= d).mean()), 3)
                                 for d in THRESHOLDS_M},
        'cones_per_frame': pct(per_m, (5, 50, 95)),
        'metres_under_2_cones': round(float((per_m < 2).sum() * GRID_M), 1),
    }
    if trace:
        k = int(round(TRACE_M / GRID_M))
        out['trace'] = {'s_m': [round(float(v), 1) for v in grid[::k]],
                        'known_ahead_m': [round(float(v), 1) for v in ahead[::k]],
                        'x_m': [round(float(v), 2) for v in frames[idx[::k], 1]],
                        'y_m': [round(float(v), 2) for v in frames[idx[::k], 2]]}
    return out


def analyse(track, spec, cones, frames, npts, nring, rng, rules):
    s_cone = path_position(cones, frames)
    L, closed = lap_of(frames)
    out = {'track': track, 'lidar': spec, 'cones': len(cones),
           'cones_on_path': int(np.isfinite(s_cone).sum()),
           'frames': len(frames), 'distance_m': round(float(frames[-1, 4]), 1),
           'lap_m': round(L, 1), 'closed': bool(closed), 'rules': {}}
    for p, k in rules:
        out['rules'][f'{p}:{k}'] = stats_for((npts >= p) & (nring >= k), rng, frames, s_cone,
                                             trace=True)
    out['point_sweep'] = {}
    for p in POINT_SWEEP:
        st = stats_for(npts >= p, rng, frames, s_cone)
        out['point_sweep'][str(p)] = {'detected': st['detected'],
                                      'first_detect_m': st['first_detect_m'],
                                      'known_ahead_m': st['known_ahead_m']}
    bands = []
    for lo, hi in zip(RANGE_BINS[:-1], RANGE_BINS[1:]):
        m = (rng >= lo) & (rng < hi)
        bands.append({'range_m': f'{lo}-{hi}', 'cone_frames': int(m.sum()),
                      'mean_points': round(float(npts[m].mean()), 2) if m.any() else None,
                      'mean_rings': round(float(nring[m].mean()), 2) if m.any() else None})
    out['points_by_range'] = bands
    return out


def load_run(run_dir):
    """A Gazebo run of cone_coverage (frames.npz + run.json) as analyse() inputs."""
    z = np.load(os.path.join(run_dir, 'frames.npz'))
    with open(os.path.join(run_dir, 'run.json')) as f:
        info = json.load(f)
    pose = z['pose']
    order = np.argsort(pose[:, 0], kind='stable')   # by sim time
    pose = pose[order]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pose[:, 1:3], axis=0).T))])
    frames = np.column_stack([pose[:, 0] - pose[0, 0], pose[:, 1], pose[:, 2], pose[:, 3], s])
    return (info, z['cones'], frames, z['npts'][order].astype(int),
            z['nring'][order].astype(int), z['rng'][order].astype(float),
            bool(z['complete']))


def summary_line(r):
    return '  '.join(f"{k}: det {v['detected']}/{r['cones']} "
                     f"first p50 {v['first_detect_m']['p50'] if v['first_detect_m'] else '-'} m "
                     f"known p5/p50 {v['known_ahead_m']['p5']}/{v['known_ahead_m']['p50']} m"
                     for k, v in r['rules'].items())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', default=None,
                    help='Gazebo runs (cone_coverage folders with frames.npz) to analyse '
                         'instead of the offline drive')
    ap.add_argument('--repeats', default=None,
                    help='folder of repeat laps (<lidar>_s<seed>/localization.npz, from '
                         'scripts/localization_repeats.sh): their localisation numbers are '
                         'added to each lidar as `localization.laps`')
    ap.add_argument('--variant', nargs=2, action='append', default=[], metavar=('NAME', 'DIR'),
                    help='laps of a variant of the stack (same layout as --repeats, e.g. a '
                         'stricter detector): added as `localization.variants.NAME`')
    ap.add_argument('--tracks', nargs='+', default=['small_track'])
    ap.add_argument('--lidars', nargs='+',
                    default=['hesai_atx', 'velodyne_vlp16@0.70', 'ouster_os1_64@0.70'])
    ap.add_argument('--vehicle', default='ads_dv')
    ap.add_argument('--rules', nargs='+', default=['3:1', '10:2', '20:3'],
                    help='detection rules as points:rings')
    ap.add_argument('--speed', type=float, default=3.0, help='target speed of the drive, m/s')
    ap.add_argument('--cone-shape', default='fs', choices=['fs', 'eufs'])
    ap.add_argument('--max-distance', type=float, default=2000.0)
    ap.add_argument('--out', default=os.path.join(os.path.expanduser(
        os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')), 'lidar_stats'))
    a = ap.parse_args()

    rules = [tuple(int(v) for v in r.split(':')) for r in a.rules]
    results = []
    if a.runs:
        for run in a.runs:
            info, cones, frames, npts, nring, rng, complete = load_run(run)
            r = analyse(info.get('track'), os.path.basename(os.path.normpath(run)), cones,
                        frames, npts, nring, rng, rules)
            r.update(source='gazebo', complete=complete, lidar_name=info['lidar']['name'],
                     lidar_mount=info.get('lidar_mount'),
                     range_max_m=info['lidar'].get('range_max_m'),
                     target_speed=info.get('target_speed'),
                     duration_s=round(float(frames[-1, 0]), 1))
            loc = localization.load(run)
            if loc is not None:
                if a.repeats:
                    loc['laps'] = localization.laps(
                        [run] + sorted(glob.glob(os.path.join(a.repeats, r['lidar'] + '_s*'))))
                for name, vdir in a.variant:
                    v = localization.laps(sorted(glob.glob(os.path.join(vdir, r['lidar'] + '_s*'))))
                    if v['n']:
                        loc.setdefault('variants', {})[name] = v
                r['localization'] = loc
            results.append(r)
            print(f"{r['lidar']:28s} {r['lap_m']:.0f} m lap, {len(frames)} scans"
                  f"{'' if complete else ' (INCOMPLETE LAP)'}  {summary_line(r)}",
                  file=sys.stderr)
            if loc is not None:
                print(f"{'':28s} localisation: {localization.summary_line(loc)}", file=sys.stderr)
    else:
        from falcon_description import config as cfg
        vehicle = cfg.load_vehicle(a.vehicle)
        for track in a.tracks:
            cones, start = load_track(track)
            frames = drive(cones, start, vehicle, a.speed, a.max_distance)
            print(f'{track}: {len(cones)} cones, drove {frames[-1, 4]:.0f} m in '
                  f'{frames[-1, 0]:.0f} s', file=sys.stderr)
            for spec in a.lidars:
                npts, nring, rng = cast_all(cones, frames, parse_lidar(spec, vehicle), vehicle,
                                            a.cone_shape)
                r = analyse(track, spec, cones, frames, npts, nring, rng, rules)
                r['source'] = 'offline'
                results.append(r)
                print(f'  {spec:32s} {summary_line(r)}', file=sys.stderr)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, 'lidar_stats.json')
    with open(path, 'w') as f:
        json.dump({'settings': {k: v for k, v in vars(a).items()}, 'results': results}, f,
                  indent=1)
    print(f'Written {path}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
