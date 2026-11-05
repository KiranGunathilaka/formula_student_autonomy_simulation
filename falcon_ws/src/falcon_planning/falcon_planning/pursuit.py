"""
Pure pursuit geometry (no ROS imports, so it can be tested offline).

Points are in the vehicle frame of ``base_footprint`` (x forward, y left).  EUFS puts
``base_footprint`` under the middle of the wheelbase, but pure pursuit's steering law
is exact about the rear axle, so the path is shifted by ``rear_axle_x`` (negative:
the rear axle is behind ``base_footprint``) before the law is applied.

The old controller aimed at the nearest waypoint beyond the lookahead distance,
anywhere on the path.  With waypoints 3-5 m apart that point could be far past the
lookahead circle or across a corner, so the car cut corners.  This one walks the
path from its closest point to the car and takes the exact point where the path
leaves the lookahead circle.
"""

import math

import numpy as np


def lookahead_point(path, L, origin=(0.0, 0.0)):
    """First point at distance L from `origin` along the path, after its closest point.

    path: (N, 2).  Returns (x, y) or None.  If the whole path is inside the circle,
    returns its last point (so the car still heads for the end of a short path).
    """
    if path is None or len(path) == 0:
        return None
    o = np.asarray(origin, dtype=float)
    rel = path - o
    d = np.hypot(rel[:, 0], rel[:, 1])
    start = int(np.argmin(d))
    for i in range(start, len(path) - 1):
        a, b = rel[i], rel[i + 1]
        if d[i] < L <= d[i + 1]:
            # Solve |a + t (b - a)| = L for t in [0, 1].
            ab = b - a
            qa = ab @ ab
            qb = 2.0 * (a @ ab)
            qc = a @ a - L * L
            disc = max(0.0, qb * qb - 4.0 * qa * qc)
            t = (-qb + math.sqrt(disc)) / (2.0 * qa) if qa > 1e-12 else 1.0
            t = min(1.0, max(0.0, t))
            pt = a + t * ab + o
            return float(pt[0]), float(pt[1])
    last = path[-1]
    if last[0] - o[0] > 0.0 or len(path) == 1:
        return float(last[0]), float(last[1])
    return None


def steering_angle(target, wheelbase, rear_axle_x=0.0):
    """Pure pursuit steering angle (rad) to reach `target` from the rear axle."""
    tx = target[0] - rear_axle_x
    ty = target[1]
    ld2 = tx * tx + ty * ty
    if ld2 < 1e-6:
        return 0.0
    # delta = atan(2 L sin(alpha) / ld) == atan(2 L y / ld^2)
    return math.atan2(2.0 * wheelbase * ty, ld2)


def speed_for_path(path, v_max, v_min, max_lat_accel, preview_m, curvature_fn):
    """Highest speed that keeps lateral acceleration under the limit over the preview."""
    if path is None or len(path) < 3 or max_lat_accel <= 0.0:
        return v_max
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    k = curvature_fn(path)
    k_max = float(np.max(k[s <= preview_m])) if np.any(s <= preview_m) else 0.0
    if k_max < 1e-6:
        return v_max
    return max(v_min, min(v_max, math.sqrt(max_lat_accel / k_max)))
