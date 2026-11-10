"""lidar_stats casting from a turned car (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_evaluation import lidar_stats

LIDAR = {'name': 'test', 'horizontal': {'fov_deg': 120.0, 'samples': 600},
         'channels_deg': list(np.linspace(-10, 10, 101)), 'range': {'min_m': 0.3, 'max_m': 100.0}}
VEHICLE = {'occluder_box': None}
MOUNT = {'x': 0.0, 'y': 0.0, 'z': 0.5, 'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0}


def test_cone_ahead_is_hit_whichever_way_the_car_points():
    for yaw in (0.0, math.pi / 2, math.pi, -2.0):
        cones = np.array([[10 * math.cos(yaw), 10 * math.sin(yaw), 1.0],      # ahead
                          [-10 * math.cos(yaw), -10 * math.sin(yaw), 2.0]])   # behind
        frames = np.array([[0.0, 0.0, 0.0, yaw, 0.0]])
        npts, nring, rng = lidar_stats.cast_all(cones, frames, [(LIDAR, MOUNT)], VEHICLE, 'fs')
        assert npts[0, 0] > 10 and nring[0, 0] >= 3, yaw
        assert npts[0, 1] == 0 and not np.isfinite(rng[0, 1]), yaw


def _circle_lap(n_frames=400, radius=30.0):
    """One anticlockwise lap of a circle, frames (t, x, y, yaw, s), and its length."""
    th = np.linspace(0, 2 * np.pi, n_frames, endpoint=False)
    x, y = radius * np.sin(th), radius * (1 - np.cos(th))
    s = radius * th
    return np.column_stack([th, x, y, th, s]), 2 * np.pi * radius


def test_known_ahead_wraps_round_a_loop_and_is_not_capped():
    frames, L = _circle_lap()
    th = np.linspace(0, 2 * np.pi, 60, endpoint=False)
    cones = np.column_stack([31.5 * np.sin(th), 30 - 31.5 * np.cos(th), np.ones(60)])
    s_cone = lidar_stats.path_position(cones, frames)
    det = np.zeros((len(frames), len(cones)), dtype=bool)
    det[0, :] = True                                  # the whole track known at once
    _, ahead, whole, _ = lidar_stats.known_ahead(det, frames, s_cone)
    assert whole.all() and np.allclose(ahead, L, rtol=0.02)     # whole lap, > 80 m
    det[:] = False
    det[0, s_cone < 0.4 * L] = True                   # the first 40 % of the lap known
    _, ahead, whole, _ = lidar_stats.known_ahead(det, frames, s_cone)
    assert not whole.any()
    assert abs(ahead[0] - 0.4 * L) < 4.0              # first unknown cone at 40 %
    # Near the end of the lap the next unknown cone is the one at 40 % of the lap: the
    # distance wraps past the start, over the known 0-40 %.
    assert abs(ahead[-1] - (L - frames[-1, 4] + 0.4 * L)) < 4.0
    stats = lidar_stats.stats_for(det, np.ones(det.shape), frames, s_cone, trace=True)
    assert stats['known_ahead_m']['max'] > 0.39 * L
    assert len(stats['trace']['s_m']) == len(stats['trace']['known_ahead_m'])
