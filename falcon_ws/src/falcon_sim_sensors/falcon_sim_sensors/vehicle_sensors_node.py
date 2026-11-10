"""
Simulated ADS-DV CAN and MTi-8 IMU, from the simulator's true state.

The stack's localisation reads only what the real car gives it, so in simulation
this node stands in for two drivers:

  /ros_can/wheel_speeds  eufs_msgs/WheelSpeedsStamped, 100 Hz: four wheel speeds in
                         whole rpm and the steer angle in rad (left positive), as the
                         EUFS / Bristol ros_can bridge publishes them from the ADS-DV's
                         VCU2AI_Speeds and VCU2AI_Steer messages
  /imu/data              sensor_msgs/Imu, every true-state message (200 Hz): MTi-8
                         angular rate and specific force with its noise and bias, no
                         orientation (the Xsens ROS 2 driver's topic)

Inputs (EUFS race car plugin): /ground_truth/state (body velocities, 200 Hz) and
/ground_truth/wheel_speeds (its `steering` is the applied steer angle).

Sources and assumptions for the numbers are in models.py and in the parameters
below.  `rolling_radius_error` is an assumption: the tyre's effective rolling radius
is not the nominal one (the EUFS wheel mesh is 505 mm, the ADS-DV CAD tyre 515 mm,
2 % apart); +1 % means the true radius is 1 % larger than the radius the localisation
uses, so it reads speeds 1 % low.  Every run uses the same `seed`, so two runs with
different lidars get the same sensor noise.
"""

import math

import numpy as np
import rclpy
from eufs_msgs.msg import CarState, WheelSpeedsStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from falcon_sim_sensors import models


class VehicleSensorsSim(Node):
    def __init__(self):
        super().__init__('vehicle_sensors_sim')
        p = self.declare_parameter
        p('wheelbase', 1.53)                 # ADS-DV Specifications & Dimensions, section 5
        p('track', 1.20)
        p('wheel_radius', 0.2525)            # nominal (EUFS wheel mesh 505 mm)
        p('rolling_radius_error', 0.01)      # assumption, see the module docstring
        p('wheel_rate_hz', 100.0)            # VCU2AI_Speeds every 10 ms (SIS p. 9)
        p('rpm_step', 1.0)                   # uint16 rpm (fs-ai_api.c)
        p('steer_step_deg', 0.1)             # 0.1 deg raw scaling (fs-ai_api.c)
        p('steer_offset_deg', 0.0)
        p('imu_frame', 'imu_link')
        p('gyro_noise_density_dps', 0.003)   # MTi-8 leaflet, deg/s/sqrt(Hz)
        p('gyro_bias_stability_dph', 6.0)    # deg/h
        p('accel_noise_density_ug', 70.0)    # ug/sqrt(Hz)
        p('accel_bias_stability_ug', 40.0)   # ug
        p('bias_tau_s', 100.0)               # Gauss-Markov time constant (assumption)
        p('seed', 1)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self._L, self._t = float(g('wheelbase')), float(g('track'))
        self._lf = self._lr = self._L / 2    # base_footprint is mid-wheelbase
        self._radius_true = float(g('wheel_radius')) * (1.0 + float(g('rolling_radius_error')))
        self._wheel_period = 1.0 / float(g('wheel_rate_hz'))
        self._rpm_step = float(g('rpm_step'))
        self._steer_step = float(g('steer_step_deg')) * models.DEG
        self._steer_offset = float(g('steer_offset_deg')) * models.DEG
        self._imu_frame = g('imu_frame')
        self._rng = np.random.default_rng(int(g('seed')))
        self._imu = None
        self._imu_args = dict(
            gyro_noise_density=float(g('gyro_noise_density_dps')) * models.DEG,
            gyro_bias_stability=float(g('gyro_bias_stability_dph')) * models.DEG / 3600.0,
            accel_noise_density=float(g('accel_noise_density_ug')) * models.UG,
            accel_bias_stability=float(g('accel_bias_stability_ug')) * models.UG,
            bias_tau_s=float(g('bias_tau_s')))

        self._steer = 0.0
        self._prev = None            # (t, v_x, v_y)
        self._next_wheel_t = None

        self.create_subscription(CarState, '/ground_truth/state', self._state_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(WheelSpeedsStamped, '/ground_truth/wheel_speeds',
                                 self._steer_cb, qos_profile_sensor_data)
        self._wheel_pub = self.create_publisher(WheelSpeedsStamped, '/ros_can/wheel_speeds', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', 50)
        self.get_logger().info(
            f'ADS-DV CAN (1 rpm, 0.1 deg, 100 Hz, rolling radius '
            f'{g("rolling_radius_error"):+.1%}) and MTi-8 IMU from the true state')

    def _steer_cb(self, msg):
        self._steer = float(msg.speeds.steering)

    def _state_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v_x, v_y = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        r = msg.twist.twist.angular.z
        if self._prev is None:
            self._prev = (t, v_x, v_y)
            self._next_wheel_t = t
            return
        dt = t - self._prev[0]
        if dt <= 1e-6:
            return                    # paused or repeated state
        if self._imu is None:
            self._imu = models.ImuModel(1.0 / dt, self._rng, **self._imu_args)
        # Specific force in the body frame: dv/dt + omega x v, plus gravity on z.
        f = ((v_x - self._prev[1]) / dt - r * v_y, (v_y - self._prev[2]) / dt + r * v_x, models.G)
        self._prev = (t, v_x, v_y)
        gyro, accel = self._imu.sample((0.0, 0.0, r), f, dt)
        self._publish_imu(msg.header.stamp, gyro, accel)

        if t + 1e-6 >= self._next_wheel_t:
            self._next_wheel_t = max(self._next_wheel_t + self._wheel_period, t)
            self._publish_wheels(msg.header.stamp, v_x, v_y, r)

    def _publish_imu(self, stamp, gyro, accel):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = self._imu_frame
        m.orientation_covariance[0] = -1.0          # no orientation
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(v) for v in gyro)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (
            float(v) for v in accel)
        gs, as_ = self._imu.gyro_sigma ** 2, self._imu.accel_sigma ** 2
        m.angular_velocity_covariance = [gs, 0.0, 0.0, 0.0, gs, 0.0, 0.0, 0.0, gs]
        m.linear_acceleration_covariance = [as_, 0.0, 0.0, 0.0, as_, 0.0, 0.0, 0.0, as_]
        self._imu_pub.publish(m)

    def _publish_wheels(self, stamp, v_x, v_y, r):
        speeds = models.wheel_speeds(v_x, v_y, r, self._steer, self._L, self._t,
                                     self._lf, self._lr)
        rpm = [abs(models.quantize(models.to_rpm(s, self._radius_true), self._rpm_step))
               for s in speeds]
        m = WheelSpeedsStamped()
        m.header.stamp = stamp
        m.header.frame_id = 'base_footprint'
        m.speeds.steering = float(models.quantize(self._steer + self._steer_offset,
                                                  self._steer_step))
        m.speeds.lf_speed, m.speeds.rf_speed, m.speeds.lb_speed, m.speeds.rb_speed = (
            float(v) for v in rpm)
        self._wheel_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSensorsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
