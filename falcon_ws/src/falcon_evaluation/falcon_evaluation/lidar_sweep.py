"""
lidar_sweep: drive the same lap with several lidar set-ups and compare cone coverage.

  ros2 run falcon_evaluation lidar_sweep sweep.yaml            # run, then tabulate
  ros2 run falcon_evaluation lidar_sweep sweep.yaml --dry-run  # print the commands
  ros2 run falcon_evaluation lidar_sweep --table <sweep dir>   # tabulate finished runs

Each run is `ros2 launch falcon_bringup sim.launch.py` headless (no Gazebo GUI, no
RViz, no camera), driven by the planner on the simulator's own cones (cones:=sim), so
the trajectory does not depend on the lidar under test.  Coverage is ray-cast from
the true pose (coverage_source:=raycast), so the lidar is not simulated in Gazebo
(use_lidar:=false); set `use_lidar: true, coverage_source: pointcloud` in the sweep
file to count Gazebo's own lidar points instead.  The coverage node exits
when the lap is done, which ends the launch; a run that overruns --timeout is
stopped and keeps a partial report.

Sweep file (see config/sweep_example.yaml):

  common: {vehicle: ads_dv, track: small_track, laps: 1, target_speed: 3.0}
  runs:                       # explicit runs ...
    - {name: atx_plate, lidar: hesai_atx}
    - {name: vlp16_post, lidar: velodyne_vlp16, lidar_z: 0.70}
  grid:                       # ... and/or every combination of these
    lidar: [robosense_helios_1615, ouster_os1_32]
    lidar_z: [0.57, 0.70]

Any sim.launch.py argument can appear in `common`, a run, or the grid.  With
`use_lidar: true, coverage_source: both` each run reports both, as <run>/raycast and
<run>/pointcloud, and the table gets a row for each.
Results: <out>/<sweep name>/<run>/{summary.json,cones.csv,run.json} and
<out>/<sweep name>/sweep_summary.{csv,md}.
"""

import argparse
import csv
import datetime
import itertools
import json
import os
import signal
import subprocess
import sys
import time

import yaml


def expand_runs(spec):
    common = dict(spec.get('common', {}))
    runs = [dict(r) for r in spec.get('runs', [])]
    grid = spec.get('grid')
    if grid:
        keys = list(grid)
        for combo in itertools.product(*(grid[k] if isinstance(grid[k], list) else [grid[k]]
                                         for k in keys)):
            runs.append(dict(zip(keys, combo)))
    out = []
    for r in runs:
        args = dict(common)
        args.update(r)
        name = args.pop('name', None) or '_'.join(
            f'{v}' if k == 'lidar' else f'{k.replace("lidar_", "")}{v}'
            for k, v in r.items() if k != 'name')
        out.append((name.replace('/', '-').replace(' ', ''), args))
    return out


def launch_cmd(args, run_dir_parent, name):
    fixed = {
        'mode': 'autonomy', 'cones': 'sim', 'use_camera': 'false', 'use_lidar': 'false',
        'localization': 'ground_truth',     # coverage sweeps judge the lidar, not the pose
        'gazebo_gui': 'false', 'rviz': 'false',
        'coverage': 'true', 'coverage_exit_on_finish': 'true',
        'coverage_output_dir': run_dir_parent, 'coverage_run_name': name,
    }
    merged = dict(fixed)
    merged.update({k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()})
    return ['ros2', 'launch', 'falcon_bringup', 'sim.launch.py'] + \
        [f'{k}:={v}' for k, v in merged.items()]


def run_one(cmd, timeout_s, log_path):
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            proc.wait(timeout=timeout_s)
            return 'finished'
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            return 'timeout'
        finally:
            # gzserver can outlive its launch; it is in our process group.
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def tabulate(sweep_dir):
    rows = []
    runs = []
    for name in sorted(os.listdir(sweep_dir)):
        d = os.path.join(sweep_dir, name)
        runs.append((name, d))
        # coverage_source: both writes <run>/raycast and <run>/pointcloud
        runs += [(f'{name}/{sub}', os.path.join(d, sub)) for sub in ('raycast', 'pointcloud')]
    for name, d in runs:
        sf, rf = os.path.join(d, 'summary.json'), os.path.join(d, 'run.json')
        if not os.path.isfile(sf):
            continue
        s = json.load(open(sf))
        r = json.load(open(rf)) if os.path.isfile(rf) else {}
        lidar = r.get('lidar', {})
        mount = r.get('lidar_mount', {})
        rec = {b['range_m']: b['detected_fraction'] for b in s['recall_by_range']}
        rows.append({
            'run': name,
            'lidar': lidar.get('name', ''),
            'channels': lidar.get('channels', ''),
            'z_m': mount.get('z', ''),
            'pitch_deg': mount.get('pitch_deg', ''),
            'source': s.get('source', ''),
            'complete': s.get('complete', False),
            'cones': s['cones'],
            'detected': s['detected'],
            'detected_%': round(100 * (s['detected_fraction'] or 0), 1),
            'too_few_points': s['hit_only'],
            'in_view_no_points': s['missed'],
            'never_in_view': s['never_in_view'],
            'first_detect_median_m': s['first_detect_range_m']['median'],
            'first_detect_p10_m': s['first_detect_range_m']['p10'],
            'recall_10_15m': rec.get('10-15'),
            'recall_15_20m': rec.get('15-20'),
            'recall_20_25m': rec.get('20-25'),
            'cloud_points_per_scan': s.get('cloud_points_per_scan'),
        })
    if not rows:
        print(f'No finished runs in {sweep_dir}')
        return
    with open(os.path.join(sweep_dir, 'sweep_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    keys = list(rows[0])
    md = ['| ' + ' | '.join(keys) + ' |', '|' + '---|' * len(keys)]
    md += ['| ' + ' | '.join('' if r[k] is None else str(r[k]) for k in keys) + ' |' for r in rows]
    open(os.path.join(sweep_dir, 'sweep_summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))
    print(f'\nWritten {sweep_dir}/sweep_summary.csv and .md')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sweep', nargs='?', help='sweep YAML')
    ap.add_argument('--out', default=os.path.expanduser(
        os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')))
    ap.add_argument('--name', default='', help='sweep folder name (default: file name + time)')
    ap.add_argument('--timeout', type=float, default=1200.0, help='wall seconds per run')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--table', metavar='SWEEP_DIR', help='only tabulate finished runs')
    a = ap.parse_args()

    if a.table:
        tabulate(a.table)
        return 0
    if not a.sweep:
        ap.error('give a sweep YAML or --table')

    spec = yaml.safe_load(open(a.sweep))
    runs = expand_runs(spec)
    stem = os.path.splitext(os.path.basename(a.sweep))[0]
    sweep_name = a.name or f"{stem}_{datetime.datetime.now():%Y%m%d_%H%M}"
    sweep_dir = os.path.join(a.out, sweep_name)
    os.makedirs(sweep_dir, exist_ok=True)
    print(f'{len(runs)} runs -> {sweep_dir}')

    for i, (name, args) in enumerate(runs, 1):
        cmd = launch_cmd(args, sweep_dir, name)
        print(f'[{i}/{len(runs)}] {name}: {" ".join(cmd[4:])}', flush=True)
        if a.dry_run:
            continue
        t0 = time.time()
        os.makedirs(os.path.join(sweep_dir, name), exist_ok=True)
        result = run_one(cmd, a.timeout, os.path.join(sweep_dir, name, 'launch.log'))
        print(f'    {result} in {time.time() - t0:.0f} s', flush=True)
        time.sleep(3)   # let Gazebo release its ports
    if not a.dry_run:
        tabulate(sweep_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
