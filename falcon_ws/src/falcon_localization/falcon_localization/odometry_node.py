"""
Odometry node: the odometry EKF (odometry.py) on the car's own sensors.

  in   /imu/data               sensor_msgs/Imu (MTi-8; frame aligned with base_footprint)
       /ros_can/wheel_speeds   eufs_msgs/WheelSpeedsStamped (ADS-DV wheel rpm, steer rad)
  out  /localization/odom      nav_msgs/Odometry, odom -> base_footprint, every IMU sample
       TF odom -> base_footprint

The odom frame starts where the car stands when the node starts; it drifts.  The cone
SLAM (slam_node.py) publishes map -> odom on top of it.
"""

import math

import rclpy
from eufs_msgs.msg import WheelSpeedsStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from falcon_localization.odometry import BG, VX, VY, X, Y, YAW, OdometryEKF


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_ekf')
        p = self.declare_parameter
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('imu_topic', '/imu/data')
        p('wheels_topic', '/ros_can/wheel_speeds')
        p('odom_topic', '/localization/odom')
        p('publish_tf', True)
        p('wheelbase', 1.53)
        p('track', 1.20)
        p('wheel_radius', 0.2525)
        p('gyro_noise_density_dps', 0.003)      # MTi-8 leaflet
        p('accel_noise_density', 0.05)          # m/s^2/sqrt(Hz), MTi-8 70 ug plus model error
        p('wheel_sigma', 0.02)                  # m/s per wheel
        p('wheel_sigma_rel', 0.01)              # of the wheel's speed
        p('lateral_sigma', 0.10)                # m/s, rear axle sideways
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._odom_frame, self._base = g('odom_frame'), g('base_frame')
        self._tf_on = bool(g('publish_tf'))
        self.ekf = OdometryEKF(
            wheelbase=float(g('wheelbase')), track=float(g('track')),
            wheel_radius=float(g('wheel_radius')),
            gyro_noise_density=math.radians(float(g('gyro_noise_density_dps'))),
            accel_noise_density=float(g('accel_noise_density')),
            wheel_sigma=float(g('wheel_sigma')), wheel_sigma_rel=float(g('wheel_sigma_rel')),
            lateral_sigma=float(g('lateral_sigma')))
        self._last_t = None
        self._tf = TransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, g('odom_topic'), 50)
        self.create_subscription(Imu, g('imu_topic'), self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(WheelSpeedsStamped, g('wheels_topic'), self._wheels_cb,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"Odometry EKF: {g('imu_topic')} + {g('wheels_topic')} -> "
                               f"{g('odom_topic')}")

    def _imu_cb(self, msg):
        t = stamp_s(msg.header.stamp)
        if self._last_t is not None:
            dt = t - self._last_t
            if dt <= 0:
                return
            if dt < 0.1:
                self.ekf.predict(msg.angular_velocity.z, msg.linear_acceleration.x,
                                 msg.linear_acceleration.y, dt)
            else:
                self.ekf.gyro_z = msg.angular_velocity.z    # gap (paused sim): do not jump
        else:
            self.ekf.gyro_z = msg.angular_velocity.z
        self._last_t = t
        self._publish(msg.header.stamp)

    def _wheels_cb(self, msg):
        if self._last_t is None:
            return
        s = msg.speeds
        self.ekf.update_wheels((s.lf_speed, s.rf_speed, s.lb_speed, s.rb_speed), s.steering)

    def _publish(self, stamp):
        x, P = self.ekf.x, self.ekf.P
        qz, qw = math.sin(x[YAW] / 2), math.cos(x[YAW] / 2)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self._odom_frame
        o.child_frame_id = self._base
        o.pose.pose.position.x, o.pose.pose.position.y = float(x[X]), float(x[Y])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        cov = [0.0] * 36
        for (i, j), (a, b) in {(0, 0): (X, X), (0, 1): (X, Y), (1, 0): (Y, X), (1, 1): (Y, Y),
                               (0, 5): (X, YAW), (5, 0): (YAW, X), (1, 5): (Y, YAW),
                               (5, 1): (YAW, Y), (5, 5): (YAW, YAW)}.items():
            cov[6 * i + j] = float(P[a, b])
        cov[14] = cov[21] = cov[28] = 1e-6
        o.pose.covariance = cov
        o.twist.twist.linear.x, o.twist.twist.linear.y = float(x[VX]), float(x[VY])
        o.twist.twist.angular.z = float(self.ekf.yaw_rate)
        tcov = [0.0] * 36
        tcov[0], tcov[7], tcov[35] = float(P[VX, VX]), float(P[VY, VY]), float(P[BG, BG])
        tcov[14] = tcov[21] = tcov[28] = 1e-6
        o.twist.covariance = tcov
        self._pub.publish(o)
        if self._tf_on:
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = self._base
            tf.transform.translation.x, tf.transform.translation.y = float(x[X]), float(x[Y])
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
