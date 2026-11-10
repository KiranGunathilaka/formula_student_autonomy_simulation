"""
Path Planner Node — Map-Aware Multi-Lap Planner
------------------------------------------------
Subscribes to live perception cones and the accumulated landmark map from
cone_map_builder, plans a centreline (see centerline.py for the algorithm) and
counts laps by detecting proximity to orange start/finish cones.

Inputs:
  /falcon/fused_cones  (eufs_msgs/ConeArrayWithCovariance, base_footprint frame)
      Live fused cones from the perception pipeline (or /cones in dev mode).
  /map/cone_map        (falcon_msgs/ConeArray, map frame)
      Accumulated landmark map with all cones observed so far.

Outputs:
  /planning/path           (nav_msgs/Path, `path_frame`, map by default)
  /planning/cone_markers   (visualization_msgs/MarkerArray)
  /planning/lap_count      (std_msgs/Int32) — current completed lap count
  /planning/finished       (std_msgs/Bool, latched) — true once `total_laps` are done

The path is published in the map frame so that pure pursuit can move it into the
car's frame at control time; a body-frame path is stale by however far the car drove
since it was planned.  Without a map -> base_footprint transform it falls back to
publishing in base_footprint.

Lap counting: when the car enters then exits the zone within
`orange_detect_radius_m` of an orange cone, the lap count goes up (the first exit,
leaving the start line, is ignored).  After `total_laps` (0 = unlimited) the planner
publishes /planning/finished and keeps planning, so pure pursuit brakes to a stop while
still steering along the track (going silent instead would let the path timeout brake
with the wheels straight, off the track if the finish is not on a straight).
"""

import math

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from builtin_interfaces.msg import Duration
from eufs_msgs.msg import ConeArrayWithCovariance
from falcon_msgs.msg import Cone, ConeArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray

from falcon_planning import centerline


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('cones_topic', '/falcon/fused_cones')
        self.declare_parameter('map_topic', '/map/cone_map')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('markers_topic', '/planning/cone_markers')
        self.declare_parameter('lap_topic', '/planning/lap_count')
        self.declare_parameter('finished_topic', '/planning/finished')
        self.declare_parameter('plan_rate_hz', 10.0)
        self.declare_parameter('total_laps', 0)
        self.declare_parameter('dedup_radius_m', 1.0)
        self.declare_parameter('orange_detect_radius_m', 3.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('use_map', True)
        self.declare_parameter('live_cone_timeout_s', 0.5)
        for k, v in centerline.DEFAULTS.items():
            self.declare_parameter(k, v)

        cones_topic = self.get_parameter('cones_topic').value
        map_topic = self.get_parameter('map_topic').value
        path_topic = self.get_parameter('path_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        lap_topic = self.get_parameter('lap_topic').value
        rate_hz = float(self.get_parameter('plan_rate_hz').value)
        self._total_laps = int(self.get_parameter('total_laps').value)
        self._dedup_r = float(self.get_parameter('dedup_radius_m').value)
        self._orange_r = float(self.get_parameter('orange_detect_radius_m').value)
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._path_frame = self.get_parameter('path_frame').value
        self._use_map = bool(self.get_parameter('use_map').value)
        self._live_timeout = float(self.get_parameter('live_cone_timeout_s').value)
        self._cl_params = {k: type(v)(self.get_parameter(k).value)
                           for k, v in centerline.DEFAULTS.items()}

        self._live = None           # (receive time, (N, 3) x, y, colour) body frame
        self._map_cones = None      # (N, 3) x, y, colour in the map frame
        self._prev_path_map = None  # last path in the map frame (for continuity)

        self._lap_count = 0
        self._in_orange_zone = False
        self._ignore_first_exit = True
        self._finished = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(ConeArrayWithCovariance, cones_topic, self._live_cb, 10)
        self.create_subscription(ConeArray, map_topic, self._map_cb, 10)

        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._lap_pub = self.create_publisher(Int32, lap_topic, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._finished_pub = self.create_publisher(
            Bool, self.get_parameter('finished_topic').value, latched)
        self._finished_pub.publish(Bool(data=False))

        self.create_timer(1.0 / rate_hz, self._plan)

        laps_str = str(self._total_laps) if self._total_laps > 0 else 'unlimited'
        self.get_logger().info(
            f'Path planner ready | live: {cones_topic} | map: {map_topic} | '
            f'laps: {laps_str} | path frame: {self._path_frame}')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _live_cb(self, msg: ConeArrayWithCovariance):
        rows = []
        for cones, color in ((msg.blue_cones, Cone.COLOR_BLUE),
                             (msg.yellow_cones, Cone.COLOR_YELLOW),
                             (msg.orange_cones, Cone.COLOR_ORANGE),
                             (msg.big_orange_cones, Cone.COLOR_BIG_ORANGE)):
            rows.extend((c.point.x, c.point.y, color) for c in cones)
        self._live = (self.get_clock().now(), np.array(rows, dtype=float).reshape(-1, 3))

    def _map_cb(self, msg: ConeArray):
        rows = [(c.pose.pose.position.x, c.pose.pose.position.y, c.color)
                for c in msg.cones if c.color != Cone.COLOR_UNKNOWN]
        self._map_cones = np.array(rows, dtype=float).reshape(-1, 3)

    # ------------------------------------------------------------------ #
    # Planning loop                                                        #
    # ------------------------------------------------------------------ #

    def _plan(self):
        self._lap_pub.publish(Int32(data=self._lap_count))

        tf_map_to_base = self._lookup(self._base_frame, self._map_frame)
        cones = self._merged_cones(tf_map_to_base)
        if len(cones) == 0:
            self.get_logger().warn('No cones to plan with', throttle_duration_sec=2.0)
            return

        color = cones[:, 2]
        blue = cones[color == Cone.COLOR_BLUE, :2]
        yellow = cones[color == Cone.COLOR_YELLOW, :2]
        orange = cones[(color == Cone.COLOR_ORANGE) | (color == Cone.COLOR_BIG_ORANGE), :2]

        if not self._finished:
            self._update_lap_count(orange)

        prev_body = None
        if self._prev_path_map is not None and tf_map_to_base is not None:
            prev_body = _apply(tf_map_to_base, self._prev_path_map)

        path, left, right = centerline.plan_centerline(
            blue, yellow, self._cl_params, orange=orange, prev_path=prev_body)
        if len(path) < 2:
            self.get_logger().warn(
                f'No centreline: {len(blue)} blue, {len(yellow)} yellow cones',
                throttle_duration_sec=2.0)
            return

        frame, out = self._base_frame, path
        if self._path_frame != self._base_frame and tf_map_to_base is not None:
            out = _apply(_invert(tf_map_to_base), path)
            frame = self._path_frame
            self._prev_path_map = out
        self._path_pub.publish(self._build_path(out, frame))
        self._markers_pub.publish(self._build_markers(blue, yellow, orange, path, left, right))

    # ------------------------------------------------------------------ #
    # Cone merging: live (body frame) + map (map -> body via TF)          #
    # ------------------------------------------------------------------ #

    def _merged_cones(self, tf_map_to_base):
        live = np.zeros((0, 3))
        if self._live is not None:
            stamp, arr = self._live
            if (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= self._live_timeout:
                live = arr
        if (not self._use_map or self._map_cones is None or len(self._map_cones) == 0
                or tf_map_to_base is None):
            return live
        m = self._map_cones.copy()
        m[:, :2] = _apply(tf_map_to_base, m[:, :2])
        if len(live) == 0:
            return m
        # Drop map cones that a live cone of the same colour already covers.
        d = np.hypot(m[:, None, 0] - live[None, :, 0], m[:, None, 1] - live[None, :, 1])
        same = m[:, None, 2] == live[None, :, 2]
        dup = np.any((d < self._dedup_r) & same, axis=1)
        return np.vstack([live, m[~dup]])

    def _lookup(self, target, source):
        """(cos, sin, tx, ty) taking points from `source` to `target`, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (math.cos(yaw), math.sin(yaw),
                tf.transform.translation.x, tf.transform.translation.y)

    # ------------------------------------------------------------------ #
    # Lap counting via orange cone proximity                              #
    # ------------------------------------------------------------------ #

    def _update_lap_count(self, orange_pts):
        if len(orange_pts) == 0:
            if self._in_orange_zone:
                self._in_orange_zone = False
                self._on_orange_exit()
            return

        now_in = float(np.min(np.hypot(orange_pts[:, 0], orange_pts[:, 1]))) < self._orange_r
        if now_in and not self._in_orange_zone:
            self._in_orange_zone = True
        elif not now_in and self._in_orange_zone:
            self._in_orange_zone = False
            self._on_orange_exit()

    def _on_orange_exit(self):
        if self._ignore_first_exit:
            self._ignore_first_exit = False
            self.get_logger().info(
                'Initial departure from start/finish — not counting as a lap.')
            return
        self._lap_count += 1
        self.get_logger().info(f'Lap {self._lap_count} completed!')
        if 0 < self._total_laps <= self._lap_count:
            self.get_logger().info(f'All {self._total_laps} laps done — stopping.')
            self._finished = True
            self._finished_pub.publish(Bool(data=True))

    # ------------------------------------------------------------------ #
    # Message builders                                                     #
    # ------------------------------------------------------------------ #

    def _build_path(self, waypoints, frame):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        n = len(waypoints)
        for i, (x, y) in enumerate(waypoints):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            # Heading along the path, for display only.
            j, k = min(i + 1, n - 1), max(min(i + 1, n - 1) - 1, 0)
            yaw = math.atan2(waypoints[j][1] - waypoints[k][1],
                             waypoints[j][0] - waypoints[k][0])
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(ps)
        return path

    def _build_markers(self, blue, yellow, orange, path, left, right):
        markers = MarkerArray()
        frame = self._base_frame
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for ns, pts, rgb, scale in (('blue_cones', blue, (0.0, 0.3, 1.0), 0.25),
                                    ('yellow_cones', yellow, (1.0, 1.0, 0.0), 0.25),
                                    ('orange_cones', orange, (1.0, 0.5, 0.0), 0.3)):
            for i, (x, y) in enumerate(pts):
                markers.markers.append(_sphere(i, ns, frame, now, lifetime, x, y, *rgb, scale))

        for ns, pts, rgb, width in (('path_line', path, (0.0, 1.0, 0.5), 0.06),
                                    ('left_chain', left, (0.2, 0.4, 1.0), 0.03),
                                    ('right_chain', right, (1.0, 0.9, 0.2), 0.03)):
            if len(pts) < 2:
                continue
            line = Marker()
            line.header.stamp = now
            line.header.frame_id = frame
            line.ns = ns
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = width
            line.color.r, line.color.g, line.color.b = rgb
            line.color.a = 0.9
            line.pose.orientation.w = 1.0
            line.lifetime = lifetime
            for x, y in pts:
                line.points.append(Point(x=float(x), y=float(y), z=0.05))
            markers.markers.append(line)
        return markers


# ------------------------------------------------------------------ #
# Helpers                                                             #
# ------------------------------------------------------------------ #

def _apply(tf, pts):
    """Apply a 2D transform (cos, sin, tx, ty) to (N, 2) points."""
    c, s, tx, ty = tf
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return np.stack([c * pts[:, 0] - s * pts[:, 1] + tx,
                     s * pts[:, 0] + c * pts[:, 1] + ty], axis=1)


def _invert(tf):
    c, s, tx, ty = tf
    return (c, -s, -(c * tx + s * ty), s * tx - c * ty)


def _sphere(uid, ns, frame, stamp, lifetime, x, y, r, g, b, scale):
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.ns = ns
    m.id = uid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
    m.lifetime = lifetime
    return m


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
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
