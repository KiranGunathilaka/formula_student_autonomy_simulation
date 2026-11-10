"""
What the ADS-DV's CAN bus and an Xsens MTi-8 would report, made from the true state.

ADS-DV CAN (ADS-DV Software Interface Specification v4.0, p. 9):
  VCU2AI_Speeds (0x525, every 10 ms): the four wheel speeds as unsigned whole rpm (the
  FS-AI API reads each one as a uint16, fs-ai_api.c), so no sign and steps of
  1 rpm = 0.026 m/s on the 0.505 m wheel.
  VCU2AI_Steer (0x523, every 10 ms): the actual steer angle of the bicycle model,
  +-21 deg, in 0.1 deg steps (the API scales the raw value by 0.1).

Xsens MTi-8 (product leaflet, Movella, Feb 2025): gyroscope noise density
0.003 deg/s/sqrt(Hz) and in-run bias stability 6 deg/h; accelerometer 70 ug/sqrt(Hz)
and 40 ug; output up to 1 kHz.  White noise per sample is density * sqrt(rate); the
bias wanders as a first-order Gauss-Markov process whose spread is the in-run bias
stability.
"""

import math

import numpy as np

G = 9.80665
DEG = math.pi / 180.0
UG = 1e-6 * G


def ackermann(delta, wheelbase, track):
    """Left and right road-wheel angles for a bicycle-model angle (left positive)."""
    if abs(delta) < 1e-6:
        return delta, delta
    r = wheelbase / math.tan(delta)              # signed turn radius at the rear axle
    return (math.atan(wheelbase / (r - track / 2)), math.atan(wheelbase / (r + track / 2)))


def wheel_speeds(v_x, v_y, r, delta, wheelbase, track, l_f, l_r):
    """Rolling speed of each wheel (fl, fr, rl, rr) in m/s along its own heading.

    Body frame at the middle of the wheelbase, x forward, y left; front axle at +l_f,
    rear at -l_r, wheels track/2 to each side; v_x, v_y, r the body velocities.  The
    velocity of a wheel's contact point is (v_x - r*y, v_y + r*x); a wheel only rolls
    along its heading (the sideways part is tyre slip, which no sensor here measures).
    """
    d_l, d_r = ackermann(delta, wheelbase, track)
    h = track / 2
    fl = (v_x - r * h) * math.cos(d_l) + (v_y + r * l_f) * math.sin(d_l)
    fr = (v_x + r * h) * math.cos(d_r) + (v_y + r * l_f) * math.sin(d_r)
    rl = v_x - r * h
    rr = v_x + r * h
    return fl, fr, rl, rr


def to_rpm(speed, radius):
    return speed / (2 * math.pi * radius) * 60.0


def quantize(value, step):
    return value if step <= 0 else round(value / step) * step


class GaussMarkov:
    """First-order Gauss-Markov process with steady-state spread `sigma`."""

    def __init__(self, sigma, tau_s, rng, size=3):
        self.sigma, self.tau = float(sigma), float(tau_s)
        self.rng = rng
        self.value = rng.normal(0.0, self.sigma, size) if self.sigma > 0 else np.zeros(size)

    def step(self, dt):
        if self.sigma <= 0 or dt <= 0:
            return self.value
        a = math.exp(-dt / self.tau)
        self.value = a * self.value + self.rng.normal(0.0, self.sigma * math.sqrt(1 - a * a),
                                                      self.value.shape)
        return self.value


class ImuModel:
    """Gyroscope (rad/s) and accelerometer (m/s^2) samples with noise and bias."""

    def __init__(self, rate_hz, rng, gyro_noise_density=0.003 * DEG,
                 gyro_bias_stability=6.0 * DEG / 3600.0, accel_noise_density=70 * UG,
                 accel_bias_stability=40 * UG, bias_tau_s=100.0):
        self.rate = float(rate_hz)
        self.rng = rng
        self.gyro_sigma = gyro_noise_density * math.sqrt(self.rate)
        self.accel_sigma = accel_noise_density * math.sqrt(self.rate)
        self.gyro_bias = GaussMarkov(gyro_bias_stability, bias_tau_s, rng)
        self.accel_bias = GaussMarkov(accel_bias_stability, bias_tau_s, rng)

    def sample(self, omega, specific_force, dt):
        gb = self.gyro_bias.step(dt)
        ab = self.accel_bias.step(dt)
        gyro = np.asarray(omega, float) + gb + self.rng.normal(0.0, self.gyro_sigma, 3)
        accel = np.asarray(specific_force, float) + ab + self.rng.normal(0.0, self.accel_sigma, 3)
        return gyro, accel
