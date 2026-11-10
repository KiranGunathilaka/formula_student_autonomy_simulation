"""Sensor models (run: python3 -m pytest test/)."""
import math

import numpy as np

from falcon_sim_sensors import models

L, T = 1.53, 1.2


def test_straight_all_wheels_roll_at_body_speed():
    for s in models.wheel_speeds(4.0, 0.0, 0.0, 0.0, L, T, L / 2, L / 2):
        assert abs(s - 4.0) < 1e-12


def test_kinematic_turn_each_wheel_rolls_round_the_turn_centre():
    # No-slip turn: the turn centre lies on the rear axle line, R from its middle.
    delta, v_rear = math.radians(15.0), 3.0
    R = L / math.tan(delta)
    r = v_rear / R
    v_x = v_rear                          # body x speed is the same along the car
    v_y = r * L / 2                       # mid-wheelbase is L/2 ahead of the rear axle
    fl, fr, rl, rr = models.wheel_speeds(v_x, v_y, r, delta, L, T, L / 2, L / 2)
    assert abs(rl - r * (R - T / 2)) < 1e-9
    assert abs(rr - r * (R + T / 2)) < 1e-9
    assert abs(fl - r * math.hypot(R - T / 2, L)) < 1e-9
    assert abs(fr - r * math.hypot(R + T / 2, L)) < 1e-9


def test_rpm_quantization_is_one_rpm():
    v = 3.0
    rpm = models.to_rpm(v, 0.2525)
    assert abs(models.quantize(rpm, 1.0) - round(rpm)) < 1e-12
    assert abs(2 * math.pi * 0.2525 / 60 - 0.0264) < 1e-4      # 1 rpm in m/s


def test_imu_white_noise_matches_the_mti8_density():
    rate = 200.0
    imu = models.ImuModel(rate, np.random.default_rng(3), gyro_bias_stability=0.0,
                          accel_bias_stability=0.0)
    g = np.array([imu.sample((0, 0, 0), (0, 0, 0), 1 / rate)[0] for _ in range(20000)])
    expected = 0.003 * models.DEG * math.sqrt(rate)
    assert abs(g[:, 2].std() / expected - 1) < 0.03


def test_bias_stays_near_the_in_run_stability():
    sigma = 6.0 * models.DEG / 3600
    gm = models.GaussMarkov(sigma, 100.0, np.random.default_rng(0), size=2000)
    for _ in range(100):
        gm.step(1.0)
    assert abs(gm.value.std() / sigma - 1) < 0.1
