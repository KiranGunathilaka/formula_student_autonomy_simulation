"""Odometry EKF on synthetic drives (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_localization.odometry import BG, OdometryEKF, ackermann

L, T, RAD = 1.53, 1.2, 0.2525


def wheel_rpm(v_x, v_y, r, delta):
    d_l, d_r = ackermann(delta, L, T)
    h, lf = T / 2, L / 2
    sp = [(v_x - r * h) * math.cos(d_l) + (v_y + r * lf) * math.sin(d_l),
          (v_x + r * h) * math.cos(d_r) + (v_y + r * lf) * math.sin(d_r),
          v_x - r * h, v_x + r * h]
    return [round(s / (2 * math.pi * RAD) * 60) for s in sp]


def drive(ekf, seconds, v, delta, rng, gyro_bias=0.0, rate=200.0):
    """Kinematic (no-slip) drive at speed v and steer delta; returns the true pose."""
    r = v * math.tan(delta) / L
    v_y = r * L / 2
    x = y = yaw = 0.0
    for i in range(int(seconds * rate)):
        dt = 1 / rate
        gz = r + gyro_bias + rng.normal(0, 0.003 * math.pi / 180 * math.sqrt(rate))
        ekf.predict(gz, -r * v_y + rng.normal(0, 0.01), r * v + rng.normal(0, 0.01), dt)
        c, s = math.cos(yaw), math.sin(yaw)
        x += (v * c - v_y * s) * dt
        y += (v * s + v_y * c) * dt
        yaw += r * dt
        if i % 2 == 0:
            ekf.update_wheels(wheel_rpm(v, v_y, r, delta), delta)
    return x, y, yaw


def test_straight_line_keeps_heading_and_distance():
    ekf = OdometryEKF()
    rng = np.random.default_rng(0)
    x, y, _ = drive(ekf, 20.0, 5.0, 0.0, rng)
    assert abs(ekf.x[0] - x) < 0.5 and abs(ekf.x[1] - y) < 0.1     # 100 m
    assert abs(ekf.x[2]) < math.radians(0.2)


def test_constant_turn_closes_within_a_metre():
    ekf = OdometryEKF()
    rng = np.random.default_rng(1)
    delta = math.atan(L / 15.0)              # 15 m radius at the rear axle
    v = 3.0
    lap = 2 * math.pi * 15.0 / v
    x, y, yaw = drive(ekf, lap, v, delta, rng)
    assert math.hypot(ekf.x[0] - x, ekf.x[1] - y) < 1.0
    assert abs((ekf.x[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(1.0)


def test_standstill_learns_the_gyro_bias():
    ekf = OdometryEKF()
    rng = np.random.default_rng(2)
    drive(ekf, 3.0, 0.0, 0.0, rng, gyro_bias=0.01)
    assert abs(ekf.x[BG] - 0.01) < 1e-3
    assert abs(ekf.x[2]) < math.radians(0.5)
