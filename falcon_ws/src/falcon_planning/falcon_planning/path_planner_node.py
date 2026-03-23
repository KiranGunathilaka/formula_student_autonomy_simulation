"""
Path Planner Node — Map-Aware Multi-Lap Planner
------------------------------------------------
Subscribes to both live perception cones AND the accumulated landmark map
from cone_map_builder. Plans a centerline path through blue/yellow midpoints
and counts laps by detecting proximity to orange start/finish cones.

Inputs:
  /perception/cones_fused  (falcon_msgs/ConeArray, base_footprint frame)
      Live field-of-view cones from the bridge/fusion pipeline.
  /map/cone_map            (falcon_msgs/ConeArray, map frame)
      Accumulated landmark map with all cones observed so far. Provides full
      track visibility once the first lap is complete.

Outputs:
  /planning/path           (nav_msgs/Path, base_footprint frame)
  /planning/cone_markers   (visualization_msgs/MarkerArray)
  /planning/lap_count      (std_msgs/Int32) — current completed lap count

Algorithm:
  1. Merge live cones (already in body frame) with map cones (transformed from
     map → base_footprint via TF).
  2. Deduplicate: if a map cone is within `dedup_radius_m` of a live cone of
     the same color, keep only the live one (it's more accurate).
  3. Separate blue / yellow, compute midpoints, order along track.
  4. Detect orange/big-orange cones near the car; when the car enters then
     exits the orange zone, increment lap count.
  5. After `total_laps`, stop publishing paths → car stops.
"""

import math

import rclpy
from rclpy.node import Node

from falcon_msgs.msg import ConeArray, Cone
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration

import tf2_ros


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('cones_topic', '/perception/cones_fused')
        self.declare_parameter('map_topic', '/map/cone_map')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('markers_topic', '/planning/cone_markers')
        self.declare_parameter('lap_topic', '/planning/lap_count')
        self.declare_parameter('min_cones_per_side', 1)
        self.declare_parameter('plan_rate_hz', 10.0)
        self.declare_parameter('waypoint_ordering', 'forward_x')
        self.declare_parameter('path_extend_m', 3.0)
        self.declare_parameter('min_midpoint_x_m', -1.5)
        self.declare_parameter('total_laps', 0)
        self.declare_parameter('dedup_radius_m', 1.0)
        self.declare_parameter('orange_detect_radius_m', 5.0)
        self.declare_parameter('map_frame', 'map')

        cones_topic   = self.get_parameter('cones_topic').value
        map_topic     = self.get_parameter('map_topic').value
        path_topic    = self.get_parameter('path_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        lap_topic     = self.get_parameter('lap_topic').value
        self._min_cones     = self.get_parameter('min_cones_per_side').value
        rate_hz             = self.get_parameter('plan_rate_hz').value
        self._order_mode    = self.get_parameter('waypoint_ordering').value
        self._path_extend_m = float(self.get_parameter('path_extend_m').value)
        self._min_mx        = float(self.get_parameter('min_midpoint_x_m').value)
        self._total_laps    = self.get_parameter('total_laps').value
        self._dedup_r       = float(self.get_parameter('dedup_radius_m').value)
        self._orange_r      = float(self.get_parameter('orange_detect_radius_m').value)
        self._map_frame     = self.get_parameter('map_frame').value

        self._live_cones: ConeArray | None = None
        self._map_cones:  ConeArray | None = None

        self._lap_count = 0
        self._in_orange_zone = False
        self._finished = False

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._live_sub = self.create_subscription(
            ConeArray, cones_topic, self._live_cb, 10)
        self._map_sub  = self.create_subscription(
            ConeArray, map_topic, self._map_cb, 10)

        self._path_pub    = self.create_publisher(Path, path_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._lap_pub     = self.create_publisher(Int32, lap_topic, 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._plan)

        laps_str = str(self._total_laps) if self._total_laps > 0 else '∞'
        self.get_logger().info(
            f'Path planner ready | live: {cones_topic} | map: {map_topic} | '
            f'laps: {laps_str}')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _live_cb(self, msg: ConeArray):
        self._live_cones = msg

    def _map_cb(self, msg: ConeArray):
        self._map_cones = msg

    # ------------------------------------------------------------------ #
    # Planning loop                                                        #
    # ------------------------------------------------------------------ #

    def _plan(self):
        self._lap_pub.publish(Int32(data=self._lap_count))

        if self._finished:
            return

        merged = self._merge_cones()
        if merged is None:
            return

        blue, yellow, orange = [], [], []
        for c in merged:
            if c[2] == Cone.COLOR_BLUE:
                blue.append((c[0], c[1]))
            elif c[2] == Cone.COLOR_YELLOW:
                yellow.append((c[0], c[1]))
            elif c[2] in (Cone.COLOR_ORANGE, Cone.COLOR_BIG_ORANGE):
                orange.append((c[0], c[1]))

        self._update_lap_count(orange)

        if len(blue) < self._min_cones or len(yellow) < self._min_cones:
            self.get_logger().warn(
                f'Not enough cones to plan: {len(blue)} blue, {len(yellow)} yellow',
                throttle_duration_sec=2.0)
            return

        midpoints = self._compute_midpoints(blue, yellow)
        if not midpoints:
            return

        ordered = self._order_waypoints(midpoints)
        ordered = self._extend_path_end(ordered)
        frame = 'base_footprint'

        self._path_pub.publish(self._build_path(ordered, frame))
        self._markers_pub.publish(
            self._build_markers(blue, yellow, orange, ordered, frame))

    # ------------------------------------------------------------------ #
    # Cone merging: live (body frame) + map (map→body via TF)             #
    # ------------------------------------------------------------------ #

    def _merge_cones(self):
        """Return list of (x, y, color) in base_footprint frame, or None."""
        live_pts = []
        if self._live_cones is not None:
            for c in self._live_cones.cones:
                live_pts.append((
                    c.pose.pose.position.x,
                    c.pose.pose.position.y,
                    c.color))

        map_pts = self._transform_map_cones()

        if not live_pts and not map_pts:
            return None

        if not map_pts:
            return live_pts
        if not live_pts:
            return map_pts

        merged = list(live_pts)
        for mx, my, mc in map_pts:
            is_dup = False
            for lx, ly, lc in live_pts:
                if mc == lc and math.hypot(mx - lx, my - ly) < self._dedup_r:
                    is_dup = True
                    break
            if not is_dup:
                merged.append((mx, my, mc))

        return merged

    def _transform_map_cones(self):
        """Transform /map/cone_map cones from map frame → base_footprint."""
        if self._map_cones is None or not self._map_cones.cones:
            return []

        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                'base_footprint', self._map_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return []

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        pts = []
        for c in self._map_cones.cones:
            mx = c.pose.pose.position.x
            my = c.pose.pose.position.y
            bx = cy * mx - sy * my + tx
            by = sy * mx + cy * my + ty
            pts.append((bx, by, c.color))
        return pts

    # ------------------------------------------------------------------ #
    # Lap counting via orange cone proximity                              #
    # ------------------------------------------------------------------ #

    def _update_lap_count(self, orange_pts):
        if not orange_pts:
            if self._in_orange_zone:
                self._in_orange_zone = False
                self._lap_count += 1
                self.get_logger().info(
                    f'Lap {self._lap_count} completed!')
                if 0 < self._total_laps <= self._lap_count:
                    self.get_logger().info(
                        f'All {self._total_laps} laps done — stopping.')
                    self._finished = True
            return

        min_d = min(math.hypot(x, y) for x, y in orange_pts)
        now_in = min_d < self._orange_r

        if now_in and not self._in_orange_zone:
            self._in_orange_zone = True
        elif not now_in and self._in_orange_zone:
            self._in_orange_zone = False
            self._lap_count += 1
            self.get_logger().info(f'Lap {self._lap_count} completed!')
            if 0 < self._total_laps <= self._lap_count:
                self.get_logger().info(
                    f'All {self._total_laps} laps done — stopping.')
                self._finished = True

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

    def _order_waypoints(self, points):
        if not points:
            return []
        if self._order_mode == 'nearest_neighbor':
            return self._nearest_neighbour_order(points, 0.0, 0.0)
        return self._order_forward_x(points)

    def _order_forward_x(self, points):
        pool = [(x, y) for x, y in points if x > self._min_mx]
        if not pool:
            pool = list(points)
        return sorted(pool, key=lambda p: (p[0], p[1]))

    def _extend_path_end(self, ordered):
        if self._path_extend_m <= 0.0 or not ordered:
            return ordered
        last = ordered[-1]
        if len(ordered) >= 2:
            px, py = ordered[-2]
            lx, ly = last
            dx, dy = lx - px, ly - py
        else:
            dx, dy = 1.0, 0.0
        n = math.hypot(dx, dy)
        if n < 1e-6:
            dx, dy = 1.0, 0.0
            n = 1.0
        ex = last[0] + (dx / n) * self._path_extend_m
        ey = last[1] + (dy / n) * self._path_extend_m
        return ordered + [(ex, ey)]

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

    def _build_markers(self, blue, yellow, orange, midpoints, frame):
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

        for i, (x, y) in enumerate(orange):
            markers.markers.append(
                _sphere(i, 'orange_cones', frame, now, lifetime,
                        x, y, r=1.0, g=0.5, b=0.0, scale=0.3))

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
