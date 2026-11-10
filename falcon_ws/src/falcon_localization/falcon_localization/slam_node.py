"""
Cone SLAM node: the EKF-SLAM (slam.py) on lidar cone detections and the odometry.

  in   /falcon/lidar_cones     eufs_msgs/ConeArrayWithCovariance, base_footprint, one
                               message per lidar scan (lidar_cone_detector_3d.py)
       /localization/odom      nav_msgs/Odometry from odometry_node.py
  out  TF map -> odom          the latest correction, sent with every odometry message
                               (at its stamp, so map -> base_footprint exists up to the
                               newest odometry) and at `tf_rate_hz` until odometry runs
       /localization/pose      geometry_msgs/PoseWithCovarianceStamped, map frame, per scan
       /localization/landmarks falcon_msgs/ConeArray, map frame, 2 Hz
       /localization/landmark_points  sensor_msgs/PointCloud2, map frame, per scan at its
                               stamp: the map after that scan (x, y, z = 0 per cone, in
                               map order), for localization_eval's map-building metric
       /localization/landmark_markers  visualization_msgs/MarkerArray, 2 Hz
       /localization/status    std_msgs/String, JSON per scan: detections, matched, new
                               cones, map size

The map frame is the track's frame: the SLAM starts at `initial_pose` (the car's start
pose, which the launch reads from the track file).  Each scan is processed at its own
stamp: the odometry pose there is interpolated from the odometry messages, so a scan
waits until the odometry has passed it.
"""

import array
import collections
import json
import math
import time

import numpy as np
import rclpy
from eufs_msgs.msg import ConeArrayWithCovariance
from falcon_msgs.msg import Cone, ConeArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from falcon_localization.slam import ConeSLAM, wrap


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def compose(a, b):
    """a * b for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class ConeSlamNode(Node):
    def __init__(self):
        super().__init__('cone_slam')
        p = self.declare_parameter
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('cones_topic', '/falcon/lidar_cones')
        p('odom_topic', '/localization/odom')
        p('initial_pose', [0.0, 0.0, 0.0])     # x, y, yaw (rad) of the car in the map
        p('tf_rate_hz', 50.0)
        p('sensor_xy', [0.0, 0.0])             # lidar position in base_footprint
        p('range_sigma', 0.10)
        p('tangential_sigma', 0.05)
        p('tangential_sigma_per_m', 0.002)
        p('candidate_min_obs', 3)
        p('min_separation', 1.0)
        p('max_innovation_m', 1.0)             # farthest a detection may be from its cone
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map, self._odom, self._base = g('map_frame'), g('odom_frame'), g('base_frame')
        pose0 = tuple(float(v) for v in g('initial_pose'))
        self.slam = ConeSLAM(pose0, sensor_xy=[float(v) for v in g('sensor_xy')],
                             range_sigma=float(g('range_sigma')),
                             tangential_sigma=float(g('tangential_sigma')),
                             tangential_sigma_per_m=float(g('tangential_sigma_per_m')),
                             candidate_min_obs=int(g('candidate_min_obs')),
                             min_separation=float(g('min_separation')),
                             max_innovation_m=float(g('max_innovation_m')))
        self._map_odom = pose0                  # odom starts at the car's start pose
        self._buf = collections.deque(maxlen=4000)    # odometry (t, x, y, yaw)
        self._pending = collections.deque(maxlen=20)
        # The odom frame starts at the car's start pose (the odometry starts while the
        # car stands there), so the first scan moves the SLAM by the whole odometry up
        # to it: the car may already be rolling when the first scan gets here.
        self._last_odom_pose = (0.0, 0.0, 0.0)
        self._last_t = None
        self._ms = collections.deque(maxlen=200)       # processing time per scan

        self._tf = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/pose', 10)
        self._lm_pub = self.create_publisher(ConeArray, '/localization/landmarks', 1)
        self._mk_pub = self.create_publisher(MarkerArray, '/localization/landmark_markers', 1)
        self._pts_pub = self.create_publisher(PointCloud2, '/localization/landmark_points', 50)
        self._status_pub = self.create_publisher(String, '/localization/status', 50)
        self.create_subscription(Odometry, g('odom_topic'), self._odom_cb, 100)
        self.create_subscription(ConeArrayWithCovariance, g('cones_topic'), self._cones_cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / float(g('tf_rate_hz')), self._tf_tick)
        self.create_timer(0.5, self._publish_map)
        self.get_logger().info(
            f"Cone EKF-SLAM: {g('cones_topic')} + {g('odom_topic')} -> TF {self._map} -> "
            f"{self._odom}, start ({pose0[0]:.2f}, {pose0[1]:.2f}, "
            f"{math.degrees(pose0[2]):.1f} deg)")

    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._buf.append((stamp_s(msg.header.stamp), msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw))
        self._drain()
        self._send_tf(msg.header.stamp)

    def _cones_cb(self, msg):
        self._pending.append(msg)
        self._drain()

    def _odom_at(self, t):
        od = self._buf
        if not od or t < od[0][0] or t > od[-1][0]:
            return None
        lo, hi = 0, len(od) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if od[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = od[lo], od[hi]
        f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                wrap(a[3] + f * wrap(b[3] - a[3])))

    def _drain(self):
        while self._pending and self._buf:
            msg = self._pending[0]
            t = stamp_s(msg.header.stamp)
            if t > self._buf[-1][0]:
                return                          # odometry has not reached this scan yet
            self._pending.popleft()
            pose = self._odom_at(t)
            if pose is None or (self._last_t is not None and t <= self._last_t):
                continue
            self._process(msg, t, pose)

    def _process(self, msg, t, odom_pose):
        t0 = time.perf_counter()
        d = compose(inverse(self._last_odom_pose), odom_pose)
        self.slam.predict(*d)
        self._last_odom_pose, self._last_t = odom_pose, t
        z = [(c.point.x, c.point.y) for arr in (msg.blue_cones, msg.yellow_cones,
                                                 msg.orange_cones, msg.big_orange_cones,
                                                 msg.unknown_color_cones) for c in arr]
        stats = self.slam.update(t, np.array(z, float).reshape(-1, 2))
        est = tuple(float(v) for v in self.slam.pose)
        self._map_odom = compose(est, inverse(odom_pose))
        self._ms.append(1e3 * (time.perf_counter() - t0))
        stats.update(t=round(t, 3), ms=round(self._ms[-1], 2))
        self._status_pub.publish(String(data=json.dumps(stats)))
        self._pts_pub.publish(self._landmark_cloud(msg.header.stamp))

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map
        out.pose.pose.position.x, out.pose.pose.position.y = est[0], est[1]
        out.pose.pose.orientation.z = math.sin(est[2] / 2)
        out.pose.pose.orientation.w = math.cos(est[2] / 2)
        P = self.slam.pose_cov
        cov = [0.0] * 36
        for (i, j), (a, b) in {(0, 0): (0, 0), (0, 1): (0, 1), (1, 0): (1, 0), (1, 1): (1, 1),
                               (0, 5): (0, 2), (5, 0): (2, 0), (1, 5): (1, 2), (5, 1): (2, 1),
                               (5, 5): (2, 2)}.items():
            cov[6 * i + j] = float(P[a, b])
        out.pose.covariance = cov
        self._pose_pub.publish(out)

    def _landmark_cloud(self, stamp):
        lms = self.slam.landmarks
        pts = np.zeros((len(lms), 3), np.float32)
        pts[:, :2] = lms
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, self._map
        cloud.height, cloud.width = 1, len(lms)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 12, 12 * len(lms)
        cloud.is_dense = True
        cloud.data = array.array('B', pts.astype('<f4').tobytes())
        return cloud

    # ------------------------------------------------------------------ #
    def _tf_tick(self):
        if not self._buf:              # before the odometry runs: keep the tree joined
            self._send_tf(self.get_clock().now().to_msg())

    def _send_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._map
        tf.child_frame_id = self._odom
        x, y, yaw = self._map_odom
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._tf.sendTransform(tf)

    def _publish_map(self):
        lms = self.slam.landmarks
        now = self.get_clock().now().to_msg()
        arr = ConeArray()
        arr.header.stamp, arr.header.frame_id = now, self._map
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, (x, y) in enumerate(lms):
            c = Cone()
            c.color = Cone.COLOR_UNKNOWN
            c.id = i
            c.pose.pose.position.x, c.pose.pose.position.y = float(x), float(y)
            c.pose.pose.orientation.w = 1.0
            P = self.slam.landmark_cov(i)
            cov = [0.0] * 36
            cov[0], cov[1], cov[6], cov[7] = float(P[0, 0]), float(P[0, 1]), float(P[1, 0]), \
                float(P[1, 1])
            c.pose.covariance = cov
            c.confidence = float(min(1.0, self.slam.lm_obs[i] / 10.0))
            arr.cones.append(c)
            m = Marker()
            m.header.stamp, m.header.frame_id = now, self._map
            m.ns, m.id, m.type, m.action = 'slam_landmarks', i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.02
            m.pose.orientation.w = 1.0
            sx = max(0.15, 2 * math.sqrt(max(P[0, 0], 1e-6)) * 2)
            sy = max(0.15, 2 * math.sqrt(max(P[1, 1], 1e-6)) * 2)
            m.scale.x, m.scale.y, m.scale.z = sx, sy, 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.2, 0.9, 0.8
            ma.markers.append(m)
        self._lm_pub.publish(arr)
        self._mk_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ConeSlamNode()
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
