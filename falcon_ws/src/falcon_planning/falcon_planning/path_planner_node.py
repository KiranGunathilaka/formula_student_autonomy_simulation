"""
Path Planner Node
-----------------
Subscribes to /perception/cones_fused (falcon_msgs/ConeArray, base_footprint
frame) and computes a centerline path between blue (left) and yellow (right)
cones using midpoint pairing, then orders the waypoints with a
nearest-neighbour traversal starting from the car's position.

All computation is done in the vehicle frame (base_footprint). The car is
always at the origin (0, 0) in this frame, so no odometry is required.

Publishes:
  /planning/path          (nav_msgs/Path)         — waypoints for the controller
  /planning/cone_markers  (visualization_msgs/MarkerArray) — RViz visualisation

Algorithm summary:
  1. Separate cones into blue and yellow lists.
  2. For each yellow cone, find its nearest blue counterpart → midpoint.
  3. Order all midpoints greedily (nearest-neighbour from car at origin).
  4. Publish the ordered list as a nav_msgs/Path.
"""

import math

import rclpy
from rclpy.node import Node

from falcon_msgs.msg import ConeArray, Cone
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('cones_topic', '/perception/cones_fused')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('markers_topic', '/planning/cone_markers')
        self.declare_parameter('min_cones_per_side', 1)
        self.declare_parameter('plan_rate_hz', 10.0)

        cones_topic  = self.get_parameter('cones_topic').value
        path_topic   = self.get_parameter('path_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self._min_cones = self.get_parameter('min_cones_per_side').value
        rate_hz      = self.get_parameter('plan_rate_hz').value

        self._cone_map: ConeArray | None = None

        self._cone_sub = self.create_subscription(
            ConeArray, cones_topic, self._cone_callback, 10)

        self._path_pub    = self.create_publisher(Path, path_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._plan)

        self.get_logger().info(f'Path planner ready | cones: {cones_topic}')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _cone_callback(self, msg: ConeArray):
        self._cone_map = msg

    # ------------------------------------------------------------------ #
    # Planning loop                                                        #
    # ------------------------------------------------------------------ #

    def _plan(self):
        if self._cone_map is None:
            return

        blue = [
            (c.pose.pose.position.x, c.pose.pose.position.y)
            for c in self._cone_map.cones
            if c.color == Cone.COLOR_BLUE
        ]
        yellow = [
            (c.pose.pose.position.x, c.pose.pose.position.y)
            for c in self._cone_map.cones
            if c.color == Cone.COLOR_YELLOW
        ]

        if len(blue) < self._min_cones or len(yellow) < self._min_cones:
            self.get_logger().warn(
                f'Not enough cones to plan: {len(blue)} blue, {len(yellow)} yellow',
                throttle_duration_sec=2.0,
            )
            return

        midpoints = self._compute_midpoints(blue, yellow)
        if not midpoints:
            return

        # Car is at origin (0,0) in base_footprint frame
        ordered = self._nearest_neighbour_order(midpoints, start_x=0.0, start_y=0.0)
        # Always use base_footprint — cone_fusion relabels its output to 'odom'
        # (which doesn't exist in TF), but the data is actually in vehicle frame.
        frame = 'base_footprint'

        self._path_pub.publish(self._build_path(ordered, frame))
        self._markers_pub.publish(self._build_markers(blue, yellow, ordered, frame))

    # ------------------------------------------------------------------ #
    # Midpoint computation                                                 #
    # ------------------------------------------------------------------ #

    def _compute_midpoints(self, blue, yellow):
        midpoints = []
        used_blue = set()

        for y_pt in yellow:
            idx = min(range(len(blue)), key=lambda i: _dist(blue[i], y_pt))
            midpoints.append(_midpoint(y_pt, blue[idx]))
            used_blue.add(idx)

        for i, b_pt in enumerate(blue):
            if i not in used_blue:
                nearest_y = min(yellow, key=lambda y: _dist(y, b_pt))
                midpoints.append(_midpoint(b_pt, nearest_y))

        return midpoints

    # ------------------------------------------------------------------ #
    # Waypoint ordering                                                    #
    # ------------------------------------------------------------------ #

    def _nearest_neighbour_order(self, points, start_x=0.0, start_y=0.0):
        remaining = list(points)
        ordered   = []
        cur_x, cur_y = start_x, start_y

        while remaining:
            idx = min(range(len(remaining)),
                      key=lambda i: _dist(remaining[i], (cur_x, cur_y)))
            nxt = remaining.pop(idx)
            ordered.append(nxt)
            cur_x, cur_y = nxt

        return ordered

    # ------------------------------------------------------------------ #
    # Message builders                                                     #
    # ------------------------------------------------------------------ #

    def _build_path(self, waypoints, frame):
        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = frame

        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        return path

    def _build_markers(self, blue, yellow, midpoints, frame):
        markers  = MarkerArray()
        now      = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        for i, (x, y) in enumerate(blue):
            markers.markers.append(
                _sphere(i, 'blue_cones', frame, now, lifetime,
                        x, y, r=0.0, g=0.3, b=1.0, scale=0.25))

        for i, (x, y) in enumerate(yellow):
            markers.markers.append(
                _sphere(i, 'yellow_cones', frame, now, lifetime,
                        x, y, r=1.0, g=1.0, b=0.0, scale=0.25))

        for i, (x, y) in enumerate(midpoints):
            markers.markers.append(
                _sphere(i, 'waypoints', frame, now, lifetime,
                        x, y, r=0.0, g=1.0, b=0.5, scale=0.15))

        if len(midpoints) >= 2:
            from geometry_msgs.msg import Point
            line = Marker()
            line.header.stamp    = now
            line.header.frame_id = frame
            line.ns        = 'path_line'
            line.id        = 0
            line.type      = Marker.LINE_STRIP
            line.action    = Marker.ADD
            line.scale.x   = 0.05
            line.color.r   = 0.0
            line.color.g   = 1.0
            line.color.b   = 0.5
            line.color.a   = 0.8
            line.lifetime  = lifetime
            for x, y in midpoints:
                pt = Point()
                pt.x = x
                pt.y = y
                line.points.append(pt)
            markers.markers.append(line)

        return markers


# ------------------------------------------------------------------ #
# Helpers                                                             #
# ------------------------------------------------------------------ #

def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _midpoint(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _sphere(uid, ns, frame, stamp, lifetime, x, y, r, g, b, scale):
    m = Marker()
    m.header.stamp    = stamp
    m.header.frame_id = frame
    m.ns              = ns
    m.id              = uid
    m.type            = Marker.SPHERE
    m.action          = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0
    m.lifetime = lifetime
    return m


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
