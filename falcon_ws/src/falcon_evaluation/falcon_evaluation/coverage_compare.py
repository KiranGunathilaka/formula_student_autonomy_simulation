"""
coverage_compare: Gazebo's lidar against the ray-cast, from one coverage_source:=both run.

  ros2 run falcon_evaluation coverage_compare ~/falcon_runs/<run>     # one run
  ros2 run falcon_evaluation coverage_compare ~/falcon_runs/<sweep>   # every run in a sweep

Reads <run>/raycast and <run>/pointcloud (summary.json, cones.csv) and prints, per
run: cones detected by each, how many cones they agree on, and the points on a cone
in each range band (mean per scan in view, and the median per-cone ratio of the most
points in one scan).  A sound simulated lidar gives the same detections and a ratio
near 1 at every range; one that renders wrongly loses the near cones or puts points
where there are none.  Both counters must use the same cone shape (cone_shape:=eufs,
the Gazebo meshes) for the ratio to mean anything.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

BANDS = ((0, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 100))


def load(run_dir, source):
    d = os.path.join(run_dir, source)
    with open(os.path.join(d, 'summary.json')) as f:
        summary = json.load(f)
    with open(os.path.join(d, 'cones.csv')) as f:
        cones = list(csv.DictReader(f))
    return summary, cones


def compare(run_dir):
    """Comparison of one run as a dict (None if it is not a both-sources run)."""
    if not all(os.path.isfile(os.path.join(run_dir, s, 'summary.json'))
               for s in ('raycast', 'pointcloud')):
        return None
    (sr, cr), (sp, cp) = load(run_dir, 'raycast'), load(run_dir, 'pointcloud')
    det_r = np.array([c['status'] == 'detected' for c in cr])
    det_p = np.array([c['status'] == 'detected' for c in cp])
    rng = np.array([float(c['closest_in_view_m']) if c['closest_in_view_m'] else np.nan
                    for c in cr])
    pr = np.array([int(c['max_points']) for c in cr])
    pp = np.array([int(c['max_points']) for c in cp])
    mean_r = {b['range_m']: b['mean_points'] for b in sr['recall_by_range']}
    mean_p = {b['range_m']: b['mean_points'] for b in sp['recall_by_range']}
    bands = []
    for lo, hi in BANDS:
        m = (rng >= lo) & (rng < hi) & (pr >= 3)
        bands.append({
            'range_m': f'{lo}-{hi}', 'cones': int(m.sum()),
            'max_points_ratio_median': round(float(np.median(pp[m] / pr[m])), 2)
            if m.any() else None,
        })
    return {
        'run': os.path.basename(os.path.normpath(run_dir)),
        'complete': bool(sr.get('complete')) and bool(sp.get('complete')),
        'cones': len(cr),
        'detected_raycast': int(det_r.sum()),
        'detected_gazebo': int(det_p.sum()),
        'agree': int((det_r == det_p).sum()),
        'first_detect_median_m': (sr['first_detect_range_m']['median'],
                                  sp['first_detect_range_m']['median']),
        'cloud_points_per_scan': sp.get('cloud_points_per_scan'),
        'mean_points': {k: (mean_r.get(k), mean_p.get(k)) for k in mean_r
                        if mean_r.get(k) is not None},
        'bands': bands,
    }


def print_one(c):
    print(f"{c['run']}: detected raycast {c['detected_raycast']}/{c['cones']}, "
          f"gazebo {c['detected_gazebo']}/{c['cones']}, agree on {c['agree']}/{c['cones']}; "
          f"median first detection {c['first_detect_median_m'][0]} / "
          f"{c['first_detect_median_m'][1]} m; {c['cloud_points_per_scan']} points per "
          f"Gazebo frame{'' if c['complete'] else '  (lap not complete)'}")
    print('  mean points on a cone in view, per scan:')
    print('    range m   raycast    gazebo')
    for k, (r, p) in c['mean_points'].items():
        print(f'    {k:>7s}  {r:>8}  {p if p is not None else "-":>8}')
    print('  most points on a cone in one scan, gazebo / raycast (median over cones):')
    print('    range m   cones   ratio')
    for b in c['bands']:
        if b['cones']:
            print(f"    {b['range_m']:>7s}   {b['cones']:5d}   {b['max_points_ratio_median']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', help='a run (with raycast/ and pointcloud/) or a sweep folder')
    ap.add_argument('--json', action='store_true', help='print JSON instead')
    a = ap.parse_args()
    runs = [a.dir] if compare(a.dir) else \
        [os.path.join(a.dir, n) for n in sorted(os.listdir(a.dir))
         if os.path.isdir(os.path.join(a.dir, n))]
    out = [c for c in (compare(r) for r in runs) if c]
    if not out:
        print(f'No coverage_source:=both runs in {a.dir}', file=sys.stderr)
        return 1
    if a.json:
        print(json.dumps(out, indent=2))
    else:
        for c in out:
            print_one(c)
    return 0


if __name__ == '__main__':
    sys.exit(main())
