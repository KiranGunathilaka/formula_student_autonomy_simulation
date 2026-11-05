"""
Vehicle and lidar profiles.

A *vehicle* file (config/vehicles/<name>.yaml) holds the car's geometry, its steering
limit and where every sensor sits.  A *lidar* file (config/lidars/<name>.yaml) holds
one lidar model: rate, horizontal view, the elevation of every channel, range, noise.

Sensor poses are measured from ``base_footprint``: the ground under the middle of the
wheelbase, x forward, y left, z up.  So ``z`` is the height above the ground and
``x`` is how far ahead of the wheelbase middle.  ``pitch_deg`` > 0 tilts the sensor
down (ROS convention).
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory


POSE_KEYS = ('x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg')


def share_dir():
    return get_package_share_directory('falcon_description')


def resolve_file(kind, name_or_path):
    """`kind` is 'vehicles' or 'lidars'.  Accepts a profile name or a path to a YAML."""
    if name_or_path.endswith('.yaml') or os.sep in name_or_path:
        path = os.path.expanduser(name_or_path)
    else:
        path = os.path.join(share_dir(), 'config', kind, name_or_path + '.yaml')
    if not os.path.isfile(path):
        avail = sorted(f[:-5] for f in os.listdir(os.path.join(share_dir(), 'config', kind))
                       if f.endswith('.yaml'))
        raise FileNotFoundError(f'No {kind[:-1]} profile "{name_or_path}". '
                                f'Available: {", ".join(avail)}')
    return path


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def pose(d):
    """Fill a sensor pose dict with zeros for missing keys."""
    d = dict(d or {})
    return {k: float(d.get(k, 0.0)) for k in POSE_KEYS}


def apply_pose_overrides(p, overrides, prefix):
    """overrides: {'lidar_z': '0.7', ...}; empty strings are ignored."""
    out = dict(p)
    for k in POSE_KEYS:
        v = overrides.get(f'{prefix}_{k}', '')
        if v not in ('', None):
            out[k] = float(v)
    return out


def channel_angles(vertical):
    """Elevation of every channel in degrees, sorted from lowest to highest."""
    if 'angles_deg' in vertical:
        angles = [float(a) for a in vertical['angles_deg']]
    elif 'uniform' in vertical:
        u = vertical['uniform']
        n = int(u['channels'])
        lo, hi = float(u['min_deg']), float(u['max_deg'])
        angles = [lo] if n == 1 else [lo + (hi - lo) * i / (n - 1) for i in range(n)]
    else:
        raise ValueError('lidar vertical: give `angles_deg` or `uniform`')
    return sorted(angles)


def uniform_groups(angles, tol_deg=0.02):
    """Split sorted channel angles into runs of equal spacing.

    Gazebo's ray sensor only spaces its vertical samples evenly, so a lidar with an
    uneven channel table is simulated as one ray sensor per run, and the clouds are
    merged.  Returns a list of (min_deg, max_deg, count).
    """
    groups = []
    i = 0
    n = len(angles)
    while i < n:
        j = i + 1
        if j < n:
            step = angles[j] - angles[i]
            while j + 1 < n and abs((angles[j + 1] - angles[j]) - step) <= tol_deg:
                j += 1
            groups.append((angles[i], angles[j], j - i + 1))
            i = j + 1
        else:
            groups.append((angles[i], angles[i], 1))
            i = j
    return groups


def load_vehicle(name_or_path, overrides=None):
    path = resolve_file('vehicles', name_or_path)
    v = load_yaml(path)
    v['_file'] = path
    overrides = overrides or {}
    sensors = v.setdefault('sensors', {})
    for s in ('lidar', 'camera', 'teleop_camera', 'imu', 'gps'):
        sensors[s] = apply_pose_overrides(pose(sensors.get(s)), overrides, s)
    return v


def load_lidar(name_or_path, overrides=None):
    path = resolve_file('lidars', name_or_path)
    L = load_yaml(path)
    L['_file'] = path
    overrides = overrides or {}
    if overrides.get('lidar_h_samples', '') not in ('', None):
        L['horizontal']['samples'] = int(overrides['lidar_h_samples'])
    if overrides.get('lidar_rate_hz', '') not in ('', None):
        L['rate_hz'] = float(overrides['lidar_rate_hz'])
    L['channels_deg'] = channel_angles(L['vertical'])
    L['groups'] = uniform_groups(L['channels_deg'])
    L['horizontal'].setdefault('centre_deg', 0.0)
    return L


def lidar_summary(L):
    ch = L['channels_deg']
    h = L['horizontal']
    gaps = [b - a for a, b in zip(ch, ch[1:])]
    finest = min(gaps) if gaps else 0.0
    rays = len(ch) * int(h['samples'])
    return (f"{L.get('name', '?')}: {len(ch)} channels {ch[0]:+.2f}..{ch[-1]:+.2f} deg "
            f"(finest {finest:.3f} deg), {h['fov_deg']:.0f} deg x {int(h['samples'])} columns, "
            f"{L['rate_hz']:.0f} Hz, {rays * L['rate_hz'] / 1e6:.2f} M rays/s, "
            f"{len(L['groups'])} Gazebo ray sensor(s)")


def deg(v):
    return math.radians(float(v))
