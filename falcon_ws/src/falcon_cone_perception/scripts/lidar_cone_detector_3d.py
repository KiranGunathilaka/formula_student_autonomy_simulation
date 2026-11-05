#!/usr/bin/env python3
"""
3D lidar cone detector (PointCloud2 -> cone positions, colour unknown).

Works for any lidar profile and mount: the cloud is moved into base_footprint with
TF (so the mount height and pitch come from the URDF, not from constants here).

  1. Keep points between `ground_z_m` and `max_z_m` above the ground, within
     `max_range_m`, and outside the car's own `body_box`.
  2. Cluster on an occupancy grid of `cluster_cell_m` cells (8-connected).
  3. Keep clusters whose footprint fits a cone (<= `max_cluster_size_m`) and whose
     top is cone-high; a top above `big_cone_min_height_m` marks a big orange cone.
  4. Shift each centroid `centroid_offset_m` away from the lidar: the points lie on
     the near face of the cone, so their mean sits in front of the cone's axis.

Publishes eufs_msgs/ConeArrayWithCovariance on /falcon/lidar_cones (unknown colour;
cone_fuser gives them colour from the camera) and cluster markers.
"""

import math

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def cloud_xyz(msg):
    """(N, 3) float32 array of x, y, z from a PointCloud2 with float32 x/y/z fields."""
    offs = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [offs['x'], offs['y'], offs['z']],
                   'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    out = np.stack([raw['x'], raw['y'], raw['z']], axis=1)
    return out[np.isfinite(out).all(axis=1)]


def tf_matrix(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def grid_clusters(xy, cell):
    """Label points by 8-connected occupied grid cells. Returns labels (N,) and count."""
    ij = np.floor(xy / cell).astype(np.int64)
    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(cells)}
    parent = list(range(len(cells)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k, (a, b) in enumerate(cells):
        for da, db in ((1, -1), (1, 0), (1, 1), (0, 1)):
            j = index.get((int(a) + da, int(b) + db))
            if j is not None:
                ra, rb = find(k), find(j)
                if ra != rb:
                    parent[rb] = ra
    roots = np.array([find(k) for k in range(len(cells))])
    _, cell_label = np.unique(roots, return_inverse=True)
    return cell_label[inv], int(cell_label.max()) + 1 if len(cells) else 0


class LidarConeDetector3D(Node):
    def __init__(self):
        super().__init__('lidar_cone_detector')
        p = self.declare_parameter
        p('points_topic', '/lidar/points')
        p('cones_topic', '/falcon/lidar_cones')
        p('base_frame', 'base_footprint')
        p('ground_z_m', 0.04)            # points lower than this are ground
        p('max_z_m', 0.70)               # higher than a big cone (0.505 m) plus margin
        p('max_range_m', 30.0)
        p('min_range_m', 0.5)
        p('body_box', [-1.35, 1.50, 0.72, 0.85])   # x_min, x_max, y_half, z_max
        p('cluster_cell_m', 0.15)
        p('max_cluster_size_m', 0.45)    # cone base is 0.228 m; big cone 0.285 m
        p('min_cluster_height_m', 0.08)
        p('big_cone_min_height_m', 0.42)  # small cone 0.325 m, big cone 0.505 m
        p('min_points', 2)
        p('centroid_offset_m', 0.05)
        p('covariance', 0.04)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._base = g('base_frame')
        self._zmin, self._zmax = float(g('ground_z_m')), float(g('max_z_m'))
        self._rmax, self._rmin = float(g('max_range_m')), float(g('min_range_m'))
        self._box = [float(v) for v in g('body_box')]
        self._cell = float(g('cluster_cell_m'))
        self._max_size = float(g('max_cluster_size_m'))
        self._min_h = float(g('min_cluster_height_m'))
        self._big_h = float(g('big_cone_min_height_m'))
        self._min_pts = int(g('min_points'))
        self._offset = float(g('centroid_offset_m'))
        self._cov = float(g('covariance'))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._static = {}   # frame -> (R, t) lidar -> base (mount is fixed)

        self.create_subscription(PointCloud2, g('points_topic'), self._cb, qos_profile_sensor_data)
        self._cone_pub = self.create_publisher(ConeArrayWithCovariance, g('cones_topic'), 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/falcon/lidar_cluster_markers', 10)
        self.get_logger().info(f"3D lidar cone detector on {g('points_topic')}")

    def _mount(self, frame):
        if frame not in self._static:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base, frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'No TF {frame} -> {self._base}: {e}',
                                       throttle_duration_sec=2.0)
                return None
            self._static[frame] = tf_matrix(tf)
        return self._static[frame]

    def _cb(self, msg):
        mount = self._mount(msg.header.frame_id)
        if mount is None:
            return
        R, t = mount
        pts = cloud_xyz(msg).astype(np.float64) @ R.T + t

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r = np.hypot(x - t[0], y - t[1])
        bx0, bx1, by, bz = self._box
        in_body = (x > bx0) & (x < bx1) & (np.abs(y) < by) & (z < bz)
        keep = (z > self._zmin) & (z < self._zmax) & (r < self._rmax) & (r > self._rmin) & ~in_body
        pts = pts[keep]

        out = ConeArrayWithCovariance()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._base
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if len(pts):
            labels, n = grid_clusters(pts[:, :2], self._cell)
            order = np.argsort(labels, kind='stable')
            bounds = np.searchsorted(labels[order], np.arange(n + 1))
            for k in range(n):
                c = pts[order[bounds[k]:bounds[k + 1]]]
                if len(c) < self._min_pts:
                    continue
                ext = c[:, :2].max(axis=0) - c[:, :2].min(axis=0)
                top = float(c[:, 2].max())
                if ext.max() > self._max_size or top < self._min_h:
                    continue
                cx, cy = c[:, 0].mean(), c[:, 1].mean()
                d = math.hypot(cx - t[0], cy - t[1])
                if d > 1e-6:
                    cx += self._offset * (cx - t[0]) / d
                    cy += self._offset * (cy - t[1]) / d
                cone = ConeWithCovariance()
                cone.point.x, cone.point.y = float(cx), float(cy)
                cone.covariance = [self._cov, 0.0, 0.0, self._cov]
                big = top > self._big_h
                out.unknown_color_cones.append(cone)
                markers.markers.append(self._marker(len(markers.markers), msg, cx, cy, top, big))

        self._cone_pub.publish(out)
        self._marker_pub.publish(markers)

    def _marker(self, i, msg, x, y, top, big):
        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self._base
        m.ns = 'lidar_cones'
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), top / 2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.22
        m.scale.z = max(top, 0.05)
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.3, 0.0, 0.8) if big else (0.8, 0.8, 0.8, 0.8)
        m.lifetime.nanosec = 200_000_000
        return m


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeDetector3D()
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


if __name__ == '__main__':
    main()
