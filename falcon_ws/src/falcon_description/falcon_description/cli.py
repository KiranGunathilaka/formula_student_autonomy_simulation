"""
falcon_urdf: list profiles, show a lidar's channel layout, or print the car URDF.

  ros2 run falcon_description falcon_urdf --list
  ros2 run falcon_description falcon_urdf --vehicle ads_dv --lidar hesai_atx --summary
  ros2 run falcon_description falcon_urdf --vehicle eufs --lidar ouster_os1_32 > car.urdf
"""

import argparse
import os
import sys

from falcon_description import config as cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vehicle', default='ads_dv')
    ap.add_argument('--lidar', default='hesai_atx')
    ap.add_argument('--list', action='store_true', help='list vehicle and lidar profiles')
    ap.add_argument('--summary', action='store_true', help='print mounts and lidar layout only')
    ap.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE',
                    help='mount overrides, e.g. lidar_z=0.7 camera_pitch_deg=10')
    a = ap.parse_args()

    if a.list:
        for kind in ('vehicles', 'lidars'):
            d = os.path.join(cfg.share_dir(), 'config', kind)
            names = sorted(f[:-5] for f in os.listdir(d) if f.endswith('.yaml'))
            print(f'{kind}: {", ".join(names)}')
        return 0

    overrides = dict(kv.split('=', 1) for kv in a.set)
    vehicle = cfg.load_vehicle(a.vehicle, overrides)
    lidar = cfg.load_lidar(a.lidar, overrides)
    if a.summary:
        print(f"vehicle: {vehicle['description']}  ({vehicle['_file']})")
        for name in ('lidar', 'camera'):
            p = vehicle['sensors'][name]
            print(f"  {name:6s} x={p['x']:+.3f} y={p['y']:+.3f} z={p['z']:.3f} m above ground, "
                  f"pitch {p['pitch_deg']:+.1f} deg (+ = down)")
        print(f"lidar: {cfg.lidar_summary(lidar)}  ({lidar['_file']})")
        for lo, hi, n in lidar['groups']:
            print(f'  ray sensor: {n:3d} channels {lo:+7.2f} .. {hi:+7.2f} deg')
        return 0

    from falcon_description.urdf_builder import build
    sys.stdout.write(build(vehicle, lidar))
    return 0


if __name__ == '__main__':
    sys.exit(main())
