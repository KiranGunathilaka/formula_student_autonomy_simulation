"""
Pure Pursuit Controller Node
-----------------------------
Subscribes to /planning/path (nav_msgs/Path) and publishes Ackermann commands.

The path may be in any frame (the planner publishes it in `map`).  Every control
step it is moved into `base_frame` with the latest TF, so the car steers from where
it is now, not from where it was when the path was planned.

Pure pursuit summary (see pursuit.py):
  1. Lookahead distance L = max(lookahead_distance, lookahead_gain * speed).
  2. Walk the path from its point closest to the car; take the exact point where it
     leaves the circle of radius L.
  3. Steer with delta = atan(2 * wheelbase * y / ld^2), measured from the rear axle
     (`rear_axle_x`, default -wheelbase / 2 because base_footprint is mid-wheelbase).
  4. Speed: target_speed, lowered so that lateral acceleration on the path ahead
     stays under max_lateral_accel; clamp steering to max_steering_angle.

Safety:
  - If no path arrives within `path_timeout_sec` (ROS time, so sim time in
    simulation), the vehicle stops.
  - Lap counting is handled by the planner; when all laps are done it stops
    publishing paths, which triggers the path timeout here.
"""

import math

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker

from falcon_planning import centerline, pursuit


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('cmd_topic', '/cmd')
        self.declare_parameter('lookahead_marker_topic', '/planning/lookahead_marker')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('lookahead_distance', 2.5)   # minimum lookahead, metres
        self.declare_parameter('lookahead_gain', 0.6)       # seconds: L grows with speed
        self.declare_parameter('wheelbase', 1.53)           # metres (ADS-DV)
        self.declare_parameter('rear_axle_x', float('nan'))  # nan: -wheelbase / 2
        self.declare_parameter('target_speed', 2.0)         # m/s
        self.declare_parameter('min_speed', 1.0)            # m/s, floor of the curve limit
        self.declare_parameter('max_lateral_accel', 3.0)    # m/s^2, 0 disables
        self.declare_parameter('speed_preview_m', 8.0)      # path ahead used for the limit
        self.declare_parameter('max_steering_angle', 0.3665)  # rad, ADS-DV AI limit (21 deg)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('path_timeout_sec', 0.5)     # stop if path stale

        path_topic = self.get_parameter('path_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        lh_topic = self.get_parameter('lookahead_marker_topic').value
        self._base_frame = self.get_parameter('base_frame').value

        self._lookahead = float(self.get_parameter('lookahead_distance').value)
        self._lh_gain = float(self.get_parameter('lookahead_gain').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        rear = float(self.get_parameter('rear_axle_x').value)
        self._rear_x = -self._wheelbase / 2.0 if math.isnan(rear) else rear
        self._target_speed = float(self.get_parameter('target_speed').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_lat = float(self.get_parameter('max_lateral_accel').value)
        self._preview = float(self.get_parameter('speed_preview_m').value)
        self._max_steer = float(self.get_parameter('max_steering_angle').value)
        self._path_timeout = float(self.get_parameter('path_timeout_sec').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self._path_pts = None     # (N, 2) in the path's own frame
        self._path_frame = None
        self._path_time = None
        self._last_speed = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, path_topic, self._path_cb, 10)
        self._cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self._lh_pub = self.create_publisher(Marker, lh_topic, 10)
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'Pure pursuit ready | L>={self._lookahead}m | v<={self._target_speed}m/s | '
            f'wb={self._wheelbase}m | steer<={self._max_steer:.3f}rad | cmd -> {cmd_topic}')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _path_cb(self, msg: Path):
        self._path_pts = np.array([(p.pose.position.x, p.pose.position.y)
                                   for p in msg.poses], dtype=float).reshape(-1, 2)
        self._path_frame = msg.header.frame_id or self._base_frame
        self._path_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # Control loop (runs at control_rate_hz)                              #
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        if self._path_pts is None or len(self._path_pts) == 0 or \
                (self.get_clock().now() - self._path_time).nanoseconds * 1e-9 > self._path_timeout:
            self._publish_cmd(0.0, 0.0)
            return

        path = self._path_in_base()
        if path is None:
            self._publish_cmd(0.0, 0.0)
            return

        L = max(self._lookahead, self._lh_gain * self._last_speed)
        lh = pursuit.lookahead_point(path, L)
        if lh is None:
            self._publish_cmd(0.0, 0.0)
            return

        steering = pursuit.steering_angle(lh, self._wheelbase, self._rear_x)
        steering = max(-self._max_steer, min(self._max_steer, steering))
        speed = pursuit.speed_for_path(path, self._target_speed, self._min_speed,
                                       self._max_lat, self._preview, centerline.path_curvature)
        self._publish_cmd(speed, steering)
        self._publish_lookahead_marker(*lh)

    def _path_in_base(self):
        if self._path_frame == self._base_frame:
            return self._path_pts
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._path_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'No TF {self._path_frame} -> {self._base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        p = self._path_pts
        return np.stack([c * p[:, 0] - s * p[:, 1] + tf.transform.translation.x,
                         s * p[:, 0] + c * p[:, 1] + tf.transform.translation.y], axis=1)

    # ------------------------------------------------------------------ #
    # Publishers                                                           #
    # ------------------------------------------------------------------ #

    def _publish_cmd(self, speed, steering):
        self._last_speed = speed
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._base_frame
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steering)
        self._cmd_pub.publish(cmd)

    def _publish_lookahead_marker(self, x, y):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._base_frame
        m.ns = 'lookahead'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.9
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        self._lh_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A callback that runs while Ctrl-C shuts the context down raises here.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
