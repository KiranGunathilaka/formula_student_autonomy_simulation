"""
Odometry EKF: IMU, four wheel speeds and the steer angle -> pose in the odom frame.

State x = [X, Y, yaw, v_x, v_y, b_g]: position and heading in `odom`, body velocities
at the middle of the wheelbase, and the gyroscope's z bias.

  predict  every IMU sample: heading from the gyro (minus its bias), body velocities
           from the accelerometer (dv/dt = f - omega x v), position from the
           velocities.
  wheels   every CAN frame: each wheel's rolling speed is the body velocity at that
           wheel along its heading (front wheels at their Ackermann angles), so the
           four wheels give v_x, a little v_y, and the yaw rate (left/right
           difference) that keeps the gyro bias honest.  Plus a soft constraint that
           the rear axle does not slide sideways.
  stand    all four wheels at 0 rpm: the car is still, so v = 0 and the gyro reads
           its own bias.
"""

import math

import numpy as np

X, Y, YAW, VX, VY, BG = range(6)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ackermann(delta, wheelbase, track):
    if abs(delta) < 1e-6:
        return delta, delta
    r = wheelbase / math.tan(delta)
    return math.atan(wheelbase / (r - track / 2)), math.atan(wheelbase / (r + track / 2))


class OdometryEKF:
    def __init__(self, *, wheelbase=1.53, track=1.2, wheel_radius=0.2525,
                 gyro_noise_density=0.003 * math.pi / 180,      # rad/s/sqrt(Hz)
                 accel_noise_density=0.05,                       # m/s^2/sqrt(Hz), incl. model
                 gyro_bias_walk=3e-6,                            # rad/s/sqrt(s)
                 wheel_sigma=0.02, wheel_sigma_rel=0.01,         # m/s, fraction of speed
                 rpm_step=1.0, lateral_sigma=0.10,               # m/s at the rear axle
                 standstill_sigma=0.005, initial_bias_sigma=0.01):
        self.L, self.t = wheelbase, track
        self.lf = self.lr = wheelbase / 2
        self.R = wheel_radius
        self.qg = gyro_noise_density ** 2
        self.qa = accel_noise_density ** 2
        self.qb = gyro_bias_walk ** 2
        self.ws, self.wr = wheel_sigma, wheel_sigma_rel
        self.quant_var = (2 * math.pi * wheel_radius / 60 * rpm_step) ** 2 / 12
        self.lat_sigma = lateral_sigma
        self.still_sigma = standstill_sigma
        self.x = np.zeros(6)
        self.P = np.diag([1e-8, 1e-8, 1e-8, 1e-4, 1e-4, initial_bias_sigma ** 2])
        self.gyro_z = 0.0             # latest gyro reading (for the yaw rate in updates)
        self.gyro_var = self.qg * 200.0   # per-sample gyro variance, set from the IMU rate
        self.yaw_rate = 0.0

    # ------------------------------------------------------------------ #
    def predict(self, gyro_z, ax, ay, dt):
        if dt <= 0:
            return
        x = self.x
        self.gyro_z = gyro_z
        self.gyro_var = self.qg / dt
        r = gyro_z - x[BG]
        self.yaw_rate = r
        c, s = math.cos(x[YAW]), math.sin(x[YAW])
        vx, vy = x[VX], x[VY]
        x[X] += (vx * c - vy * s) * dt
        x[Y] += (vx * s + vy * c) * dt
        x[YAW] = wrap(x[YAW] + r * dt)
        x[VX] += (ax + r * vy) * dt
        x[VY] += (ay - r * vx) * dt

        A = np.zeros((6, 6))
        A[X, YAW], A[X, VX], A[X, VY] = -vx * s - vy * c, c, -s
        A[Y, YAW], A[Y, VX], A[Y, VY] = vx * c - vy * s, s, c
        A[YAW, BG] = -1.0
        A[VX, VY], A[VX, BG] = r, -vy
        A[VY, VX], A[VY, BG] = -r, vx
        F = np.eye(6) + A * dt
        # Noise inputs: gyro (enters yaw and both velocity cross terms), accel x, y, bias.
        Gm = np.zeros((6, 4))
        Gm[YAW, 0], Gm[VX, 0], Gm[VY, 0] = 1.0, vy, -vx
        Gm[VX, 1] = 1.0
        Gm[VY, 2] = 1.0
        Gm[BG, 3] = 1.0
        Q = Gm @ np.diag([self.qg, self.qa, self.qa, self.qb]) @ Gm.T * dt
        self.P = F @ self.P @ F.T + Q

    def _update(self, z, h, H, Rm):
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        nu = z - h
        self.x = self.x + K @ nu
        self.x[YAW] = wrap(self.x[YAW])
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T          # Joseph form
        return float(nu @ np.linalg.solve(S, nu))

    def update_wheels(self, rpm, delta):
        """rpm: (fl, fr, rl, rr) unsigned; delta: bicycle-model steer angle (rad)."""
        rpm = np.asarray(rpm, float)
        if np.all(rpm == 0):
            return self.update_standstill()
        speeds = rpm * 2 * math.pi * self.R / 60.0          # forward driving assumed
        x = self.x
        vx, vy, r = x[VX], x[VY], self.gyro_z - x[BG]
        d_l, d_r = ackermann(delta, self.L, self.t)
        h2, lf = self.t / 2, self.lf
        cl, sl, cr, sr = math.cos(d_l), math.sin(d_l), math.cos(d_r), math.sin(d_r)
        # Rolling speed at each wheel and its derivative w.r.t. (v_x, v_y, r).
        rows = [
            ((vx - r * h2) * cl + (vy + r * lf) * sl, cl, sl, -h2 * cl + lf * sl),   # FL
            ((vx + r * h2) * cr + (vy + r * lf) * sr, cr, sr, h2 * cr + lf * sr),    # FR
            (vx - r * h2, 1.0, 0.0, -h2),                                            # RL
            (vx + r * h2, 1.0, 0.0, h2),                                             # RR
        ]
        H = np.zeros((5, 6))
        h = np.zeros(5)
        z = np.zeros(5)
        Rm = np.zeros((5, 5))
        for i, (hi, dvx, dvy, dr) in enumerate(rows):
            h[i], z[i] = hi, speeds[i]
            H[i, VX], H[i, VY], H[i, BG] = dvx, dvy, -dr      # r = gyro - b_g
            Rm[i, i] = self.ws ** 2 + (self.wr * speeds[i]) ** 2 + self.quant_var
        # Rear axle does not slide sideways: v_y - r * l_r = 0.
        h[4] = vy - r * self.lr
        H[4, VY], H[4, BG] = 1.0, self.lr
        Rm[4, 4] = self.lat_sigma ** 2
        return self._update(z, h, H, Rm)

    def update_standstill(self):
        """v_x = v_y = 0 and the gyro reads its bias."""
        x = self.x
        H = np.zeros((3, 6))
        H[0, VX], H[1, VY], H[2, BG] = 1.0, 1.0, 1.0
        h = np.array([x[VX], x[VY], x[BG]])
        z = np.array([0.0, 0.0, self.gyro_z])
        Rm = np.diag([self.still_sigma ** 2, self.still_sigma ** 2, self.gyro_var])
        return self._update(z, h, H, Rm)
