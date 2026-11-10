"""Cone EKF-SLAM on a synthetic ring of cones (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_localization.slam import ConeSLAM, wrap

R_TRACK = 20.0


def ring_cones(spacing=3.0):
    cones = []
    for r in (R_TRACK - 1.75, R_TRACK + 1.75):
        n = int(2 * math.pi * r / spacing)
        th = np.linspace(0, 2 * math.pi, n, endpoint=False)
        cones += list(zip(r * np.sin(th), R_TRACK - r * np.cos(th)))
    return np.array(cones)


def run(laps=1.2, scale_error=0.02, yaw_drift=1e-4, fov_deg=120.0, max_range=25.0, seed=0):
    rng = np.random.default_rng(seed)
    cones = ring_cones()
    slam = ConeSLAM((0.0, 0.0, 0.0))
    v, rate = 4.0, 10.0
    w = v / R_TRACK
    steps = int(laps * 2 * math.pi * R_TRACK / v * rate)
    odo = np.zeros(3)
    worst = odo_worst = 0.0
    for k in range(1, steps + 1):
        dt = 1 / rate
        # Odometry increment in the body frame: speed read 2 % low, heading drifting
        # 0.06 deg/s, a little yaw noise.
        dth = w * dt
        dx, dy = R_TRACK * math.sin(dth), R_TRACK * (1 - math.cos(dth))
        mx, my = dx * (1 - scale_error), dy * (1 - scale_error)
        myaw = dth + yaw_drift + rng.normal(0, 0.0005)
        slam.predict(mx, my, myaw)
        c, s = math.cos(odo[2]), math.sin(odo[2])
        odo += (c * mx - s * my, s * mx + c * my, myaw)
        th = w * dt * k
        truth = (R_TRACK * math.sin(th), R_TRACK * (1 - math.cos(th)), wrap(th))
        c, s = math.cos(truth[2]), math.sin(truth[2])
        rel = (cones - truth[:2]) @ np.array([[c, -s], [s, c]])
        rng_ = np.hypot(*rel.T)
        bearing = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
        seen = (rng_ < max_range) & (np.abs(bearing) < fov_deg / 2)
        z = rel[seen] + rng.normal(0, 0.03, (seen.sum(), 2))
        slam.update(k * dt, z)
        worst = max(worst, math.hypot(slam.pose[0] - truth[0], slam.pose[1] - truth[1]))
        odo_worst = max(odo_worst, math.hypot(odo[0] - truth[0], odo[1] - truth[1]))
    err = math.hypot(slam.pose[0] - truth[0], slam.pose[1] - truth[1])
    return slam, cones, err, odo_worst, worst


def test_slam_beats_odometry_and_closes_the_loop():
    slam, cones, err, odo_worst, worst = run()
    assert odo_worst > 0.8                  # dead reckoning alone drifts about a metre
    assert err < 0.3 and worst < 0.2 * odo_worst


def test_unmapped_cone_does_not_pull_the_car_to_its_neighbour():
    # Loop closure with a short-range detector: the car is metres unsure of where it is
    # relative to the cones mapped at the start, and sees a cone the map does not hold,
    # 4.5 m short of a mapped one.  The chi-square gate alone pairs them.
    def closure(**kw):
        slam = ConeSLAM((0.0, 0.0, 0.0), **kw)
        slam._add_landmark(np.array([5.0, 2.0]), np.diag([0.01, 0.01]))
        slam.P[:3, :3] = np.diag([1.5 ** 2, 1.5 ** 2, 0.03 ** 2])
        stats = slam.update(0.0, [[0.5, 2.0]])
        return stats['matched'], math.hypot(slam.pose[0], slam.pose[1])
    matched, moved = closure(max_innovation_m=1e9)
    assert matched == 1 and moved > 3.0          # without the cap the car jumps
    matched, moved = closure()
    assert matched == 0 and moved < 1e-9


def test_map_has_each_cone_once():
    slam, cones, *_ = run()
    lms = slam.landmarks
    d = np.hypot(*(lms[:, None, :] - cones[None]).transpose(2, 0, 1))
    assert abs(len(lms) - len(cones)) <= 2                     # no duplicates
    assert np.sqrt(np.mean(d.min(axis=1) ** 2)) < 0.15
