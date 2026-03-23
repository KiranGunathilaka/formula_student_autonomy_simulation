"""
Pure Pursuit Controller Node
-----------------------------
Subscribes to /planning/path (nav_msgs/Path, base_footprint frame).
The path is in the vehicle frame: the car is always at the origin (0, 0)
facing forward (+x).  No odometry is required for the geometry.

Pure Pursuit summary:
  1. Find the lookahead point: nearest waypoint at distance >= L from origin
     with a positive x component (ahead of the car).
  2. Compute the steering angle:
       delta = atan( 2 * wheelbase * sin(alpha) / L )
     where alpha = atan2(waypoint.y, waypoint.x) — heading error in vehicle frame.
  3. Clamp to [-max_steer, +max_steer] and publish with target speed.

Safety:
  - If no path arrives within `path_timeout_sec`, the vehicle stops.
  - If the last path waypoint is within `goal_tolerance_m` of origin,
    the vehicle stops (path complete).
"""

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('path_topic',             '/planning/path')
        self.declare_parameter('cmd_topic',              '/cmd')
        self.declare_parameter('lookahead_marker_topic', '/planning/lookahead_marker')

        self.declare_parameter('lookahead_distance',  2.5)    # metres
        self.declare_parameter('wheelbase',           1.53)   # metres (ADS-DV)
        self.declare_parameter('target_speed',        2.0)    # m/s
        self.declare_parameter('max_steering_angle',  0.44)   # radians (~25 deg)
        self.declare_parameter('control_rate_hz',     20.0)
        self.declare_parameter('path_timeout_sec',    0.5)    # stop if path stale
        self.declare_parameter('goal_tolerance_m',    1.0)    # stop near last wp

        path_topic    = self.get_parameter('path_topic').value
        cmd_topic     = self.get_parameter('cmd_topic').value
        lh_topic      = self.get_parameter('lookahead_marker_topic').value

        self._lookahead     = self.get_parameter('lookahead_distance').value
        self._wheelbase     = self.get_parameter('wheelbase').value
        self._target_speed  = self.get_parameter('target_speed').value
        self._max_steer     = self.get_parameter('max_steering_angle').value
        self._path_timeout  = self.get_parameter('path_timeout_sec').value
        self._goal_tol      = self.get_parameter('goal_tolerance_m').value
        rate_hz             = self.get_parameter('control_rate_hz').value

        self._path: Path | None = None
        self._path_stamp: float = 0.0

        self._path_sub = self.create_subscription(Path, path_topic, self._path_cb, 10)
        self._cmd_pub  = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self._lh_pub   = self.create_publisher(Marker, lh_topic, 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'Pure pursuit ready | L={self._lookahead}m | '
            f'v={self._target_speed}m/s | wb={self._wheelbase}m'
        )

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _path_cb(self, msg: Path):
        self._path       = msg
        self._path_stamp = time.monotonic()

    # ------------------------------------------------------------------ #
    # Control loop (runs at control_rate_hz)                              #
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        # Stop if path is stale or missing
        if self._path is None or \
                (time.monotonic() - self._path_stamp) > self._path_timeout:
            self._publish_stop()
            return

        if not self._path.poses:
            self._publish_stop()
            return

        # In base_footprint frame the car is always at the origin.
        # Check if near last waypoint → goal reached
        last = self._path.poses[-1].pose.position
        if math.hypot(last.x, last.y) < self._goal_tol:
            self.get_logger().info('Goal reached — stopping.', throttle_duration_sec=2.0)
            self._publish_stop()
            return

        lh = self._find_lookahead()
        if lh is None:
            self._publish_stop()
            return

        lx, ly = lh
        L = math.hypot(lx, ly)
        if L < 0.01:
            return

        # alpha is the angle to the lookahead point in vehicle frame
        alpha    = math.atan2(ly, lx)
        steering = math.atan2(2.0 * self._wheelbase * math.sin(alpha), L)
        steering = max(-self._max_steer, min(self._max_steer, steering))

        self._publish_cmd(self._target_speed, steering)
        self._publish_lookahead_marker(lx, ly)

    # ------------------------------------------------------------------ #
    # Lookahead search                                                     #
    # ------------------------------------------------------------------ #

    def _find_lookahead(self):
        """
        Walk the path and return the closest waypoint that is:
          - at distance >= lookahead_distance from the origin, AND
          - ahead of the car (x > 0 in vehicle frame).

        Falls back to the furthest forward waypoint if none qualify.
        """
        best      = None
        best_dist = float('inf')

        for pose in self._path.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            d  = math.hypot(wx, wy)

            if wx <= 0.0:
                continue  # behind the car

            if d < self._lookahead:
                continue  # inside lookahead circle

            if d < best_dist:
                best      = (wx, wy)
                best_dist = d

        if best is not None:
            return best

        # Fallback: furthest waypoint that is still ahead
        for pose in reversed(self._path.poses):
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            if wx > 0.0:
                return (wx, wy)

        return None

    # ------------------------------------------------------------------ #
    # Publishers                                                           #
    # ------------------------------------------------------------------ #

    def _publish_cmd(self, speed, steering):
        cmd = AckermannDriveStamped()
        cmd.header.stamp         = self.get_clock().now().to_msg()
        cmd.header.frame_id      = 'base_footprint'
        cmd.drive.speed          = speed
        cmd.drive.steering_angle = steering
        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp         = self.get_clock().now().to_msg()
        cmd.header.frame_id      = 'base_footprint'
        cmd.drive.speed          = 0.0
        cmd.drive.steering_angle = 0.0
        self._cmd_pub.publish(cmd)

    def _publish_lookahead_marker(self, x, y):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.ns              = 'lookahead'
        m.id              = 0
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 0.35
        m.color.r = 1.0
        m.color.g = 0.4
        m.color.b = 0.0
        m.color.a = 0.9
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        self._lh_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()
