"""
Cone coverage node: did the lidar see every cone of the track?

Reads the full ground-truth track (/ground_truth/track, all cones in the map frame)
and the car's pose from TF, and every lidar frame counts the returns on every cone
(see coverage.py).  When the car has driven `laps` laps (it left the start and came
back within `start_radius_m`) it writes a report and, with `exit_on_finish`, exits so
that a sweep can start the next run.

source:=raycast (default) casts the lidar profile's rays itself, `rate_hz` times a
second, from `lidar_mount` (x, y, z, roll, pitch, yaw in degrees, in base_footprint):
exact, fast, and the lidar does not have to be simulated in Gazebo at all.
source:=pointcloud counts the points of `points_topic` instead (Gazebo's lidar, or a
real one), with the mount taken from TF.  On the test PC (Intel graphics, Mesa)
Gazebo's gpu_ray lidar dropped most returns, and the exact CPU `ray` sensor ran out of
memory with the ATX, which is why raycast is the default.

The pose comes from TF map -> base_footprint, which EUFS publishes from the
simulator's state (its pose noise is zero in eufs_models/config/noise.yaml), so the
cones are placed exactly; only the lidar is under test.

Outputs, in <output_dir>/<run_name>/:
  summary.json      totals, per colour, first-detection range, recall by range
  cones.csv         one row per cone: status, points, rings, ranges
  run.json          vehicle, lidar profile, mount, track, criteria
Topics:
  /coverage/markers  every cone in the map frame: green detected, orange points but
                     not enough, red in view but no points, grey never in view
  /coverage/summary  one-line text summary (std_msgs/String)
Services:
  /coverage/write_report  (std_srvs/Trigger) write the report now
"""

import csv
import datetime
import json
import math
import os

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from eufs_msgs.msg import ConeArrayWithCovariance
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from falcon_evaluation.coverage import CoverageAccumulator

STATUS_RGB = {'detected': (0.1, 0.85, 0.2), 'hit_only': (1.0, 0.6, 0.0),
              'missed': (0.9, 0.1, 0.1), 'never_in_view': (0.5, 0.5, 0.5)}


def cloud_xyz(msg):
    offs = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [offs['x'], offs['y'], offs['z']], 'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    out = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(float)
    return out[np.isfinite(out).all(axis=1)]


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class ConeCoverageNode(Node):
    def __init__(self):
        super().__init__('cone_coverage')
        p = self.declare_parameter
        p('points_topic', '/lidar/points')
        p('track_topic', '/ground_truth/track')
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        p('channels_deg', [0.0])
        p('hfov_deg', 360.0)
        p('hcentre_deg', 0.0)
        p('max_range_m', 100.0)
        p('min_points', 3)
        p('min_rings', 1)
        p('catch_radius_m', 0.25)
        p('body_box', [0.0])         # x_min, x_max, y_half[, z_max]; [0.0] = do not check
        p('laps', 1)
        p('start_radius_m', 2.5)
        p('min_lap_distance_m', 30.0)
        p('exit_on_finish', False)
        p('output_dir', os.path.expanduser(os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')))
        p('run_name', '')
        p('run_info_json', '{}')     # vehicle / lidar / mount description from the launch
        p('source', 'raycast')       # raycast | pointcloud
        p('cone_shape', 'fs')        # fs (rulebook cones) | eufs (the Gazebo meshes)
        p('rate_hz', 10.0)           # raycast frames per second (sim time)
        p('h_samples', 1800)         # columns per frame over hfov_deg (raycast)
        p('min_range_m', 0.3)
        p('lidar_mount', [0.0])      # x, y, z, roll_deg, pitch_deg, yaw_deg (raycast)
        p('occluder_box', [0.0])     # bodywork x_min, x_max, y_half, z_max (raycast)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map, self._base = g('map_frame'), g('base_frame')
        self._laps_target = int(g('laps'))
        self._start_r = float(g('start_radius_m'))
        self._min_lap = float(g('min_lap_distance_m'))
        self._exit = bool(g('exit_on_finish'))
        self._info = json.loads(g('run_info_json') or '{}')
        name = g('run_name') or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._out = os.path.join(os.path.expanduser(g('output_dir')), name)
        self._acc_args = dict(
            channels_deg=[float(v) for v in g('channels_deg')], hfov_deg=float(g('hfov_deg')),
            hcentre_deg=float(g('hcentre_deg')), max_range_m=float(g('max_range_m')),
            min_points=int(g('min_points')), min_rings=int(g('min_rings')),
            catch_radius_m=float(g('catch_radius_m')),
            body_box=[float(v) for v in g('body_box')] if len(g('body_box')) >= 3 else None,
            cone_shape=g('cone_shape'), h_samples=int(g('h_samples')),
            min_range_m=float(g('min_range_m')),
            occluder_box=[float(v) for v in g('occluder_box')] if len(g('occluder_box')) >= 4
            else None)
        self._source = g('source')
        if self._source not in ('raycast', 'pointcloud'):
            raise ValueError(f'source:={self._source}: use raycast or pointcloud')

        self._acc = None
        self._mount = None          # (R, t) lidar -> base
        self._start = None
        self._last_pos = None
        self._travelled = 0.0
        self._away = False
        self._laps = 0
        self._done = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._track_sub = self.create_subscription(
            ConeArrayWithCovariance, g('track_topic'), self._track_cb, 1)
        if self._source == 'pointcloud':
            self.create_subscription(PointCloud2, g('points_topic'), self._cloud_cb,
                                     qos_profile_sensor_data)
        else:
            m = [float(v) for v in g('lidar_mount')]
            if len(m) != 6:
                raise ValueError('lidar_mount needs x, y, z, roll_deg, pitch_deg, yaw_deg')
            self._mount = (rpy_to_R(*(math.radians(v) for v in m[3:])), np.array(m[:3]))
            self.create_timer(1.0 / float(g('rate_hz')), self._raycast_tick)
        self._marker_pub = self.create_publisher(MarkerArray, '/coverage/markers', 1)
        self._summary_pub = self.create_publisher(String, '/coverage/summary', 1)
        self.create_service(Trigger, '/coverage/write_report', self._srv_write)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Cone coverage ({self._source}, {g('cone_shape')} cones): "
            f"{len(self._acc_args['channels_deg'])} channels, "
            f"{self._acc_args['hfov_deg']:.0f} deg view, detected = >= {self._acc_args['min_points']} "
            f"points on >= {self._acc_args['min_rings']} ring(s); report -> {self._out}")

    # ------------------------------------------------------------------ #
    def _track_cb(self, msg):
        rows = []
        for cones, code in ((msg.blue_cones, 1), (msg.yellow_cones, 2), (msg.orange_cones, 3),
                            (msg.big_orange_cones, 4), (msg.unknown_color_cones, 0)):
            rows.extend((c.point.x, c.point.y, code) for c in cones)
        if not rows:
            return
        self._acc = CoverageAccumulator(np.array(rows), **self._acc_args)
        self.get_logger().info(f'Ground-truth track: {len(rows)} cones')
        # The track does not move; stop the plugin from publishing it.
        self.destroy_subscription(self._track_sub)

    def _lookup(self, target, source, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                target, source, stamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF {source} -> {target}: {e}', throttle_duration_sec=2.0)
            return None

    def _cloud_cb(self, msg):
        if self._acc is None or self._done:
            return
        if self._mount is None:
            tf = self._lookup(self._base, msg.header.frame_id, rclpy.time.Time())
            if tf is None:
                return
            t = tf.transform.translation
            self._mount = (quat_to_R(tf.transform.rotation), np.array([t.x, t.y, t.z]))
        tf = self._lookup(self._base, self._map, rclpy.time.Time.from_msg(msg.header.stamp))
        if tf is None:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        R, t = self._mount
        self._acc.add_scan_points(cloud_xyz(msg), R, t, (c, s, tx, ty))
        # Car position in the map: invert map -> base.
        self._update_laps(np.array([-(c * tx + s * ty), s * tx - c * ty]))

    def _raycast_tick(self):
        if self._acc is None or self._done:
            return
        tf = self._lookup(self._base, self._map, rclpy.time.Time())
        if tf is None:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        pos = np.array([-(c * tx + s * ty), s * tx - c * ty])
        if self._last_pos is not None and np.hypot(*(pos - self._last_pos)) < 1e-4 \
                and self._acc.scans > 0:
            return   # sim paused or no new pose: do not count the same frame twice
        R, t = self._mount
        self._acc.add_scan_raycast(R, t, (c, s, tx, ty))
        self._update_laps(pos)

    def _update_laps(self, pos):
        if self._start is None:
            self._start = pos
        if self._last_pos is not None:
            self._travelled += float(np.hypot(*(pos - self._last_pos)))
        self._last_pos = pos
        d = float(np.hypot(*(pos - self._start)))
        if d > 2 * self._start_r:
            self._away = True
        if self._away and d < self._start_r and self._travelled > self._min_lap * (self._laps + 1):
            self._laps += 1
            self._away = False
            self.get_logger().info(f'Lap {self._laps} done after {self._travelled:.0f} m')
            if self._laps >= self._laps_target > 0:
                self._finish()

    def _finish(self):
        self._done = True
        path = self.write_report()
        self.get_logger().info(f'Coverage report written to {path}')
        self._publish_status()
        if self._exit:
            raise SystemExit(0)

    # ------------------------------------------------------------------ #
    def _summary_line(self):
        if self._acc is None:
            return 'coverage: waiting for /ground_truth/track'
        s = self._acc.summary()
        fd = s['first_detect_range_m']['median']
        name = self._info.get('lidar', {}).get('name', 'lidar')
        z = self._info.get('lidar_mount', {}).get('z')
        mount = f' z={z:.2f}m' if isinstance(z, (int, float)) else ''
        return (f"{name}{mount}: {s['detected']}/{s['cones']} cones detected"
                f" ({100 * (s['detected_fraction'] or 0):.0f}%), {s['hit_only']} too few points,"
                f" {s['missed']} in view but no points, {s['never_in_view']} never in view;"
                f" {s['cones_touched_by_car'] or 0} touched by the car;"
                f" median first-detection range {fd} m; lap {self._laps}/{self._laps_target},"
                f" {self._travelled:.0f} m, {s['scans']} scans")

    def _publish_status(self):
        self._summary_pub.publish(String(data=self._summary_line()))
        if self._acc is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (row, st) in enumerate(zip(self._acc.cones, self._acc.status())):
            m = Marker()
            m.header.frame_id = self._map
            m.header.stamp = now
            m.ns = 'coverage'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            h = self._acc.height[i]
            m.pose.position.x, m.pose.position.y, m.pose.position.z = row[0], row[1], h / 2 + 0.6
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z = h
            m.color.r, m.color.g, m.color.b = STATUS_RGB[st]
            m.color.a = 0.9
            ma.markers.append(m)
        self._marker_pub.publish(ma)

    def write_report(self):
        if self._acc is None:
            return None
        os.makedirs(self._out, exist_ok=True)
        summary = self._acc.summary()
        summary['laps_completed'] = self._laps
        summary['distance_m'] = round(self._travelled, 1)
        summary['complete'] = self._done
        with open(os.path.join(self._out, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        info = dict(self._info)
        info['criteria'] = summary['criteria']
        info['written'] = datetime.datetime.now().isoformat(timespec='seconds')
        with open(os.path.join(self._out, 'run.json'), 'w') as f:
            json.dump(info, f, indent=2)
        rows = self._acc.cone_rows()
        with open(os.path.join(self._out, 'cones.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        return self._out

    def _srv_write(self, req, resp):
        path = self.write_report()
        resp.success = path is not None
        resp.message = f'{path}: {self._summary_line()}' if path else 'no track yet'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ConeCoverageNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except RuntimeError:
        # A callback that runs while Ctrl-C shuts the context down raises here.
        if rclpy.ok():
            raise
    finally:
        if not node._done:
            # Partial report (run stopped before the lap was finished).
            path = node.write_report()
            if path:
                print(f'[cone_coverage] partial report written to {path}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
