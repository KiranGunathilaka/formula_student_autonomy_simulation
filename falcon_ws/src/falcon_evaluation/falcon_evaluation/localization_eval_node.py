"""
Localisation evaluation node: the stack's pose against the simulator's, over a lap.

Twenty times a second (sim time) it takes the stack's pose (TF map -> base_footprint,
which goes through the cone SLAM's map -> odom), the true pose at the same stamp
(TF map -> base_footprint_gt, published by the EUFS plugin when the launch runs the
localisation), and dead reckoning alone (TF odom -> base_footprint, lined up with
the truth at the first sample).  It also keeps the SLAM's per-scan status
(/localization/status), its last cone map (/localization/landmarks) and its map after
every scan (/localization/landmark_points), for the map-building metric.

When the car has driven `laps` laps (same rule as cone_coverage: away from the start
and back within `start_radius_m`) it writes, in <output_dir>/<run_name>/:
  localization.json  summary (localization.py: position, along/cross-track and
                     heading error distributions over the lap, final error, map
                     quality, detections per scan), for the estimate and for dead
                     reckoning
  localization.npz   t, truth, est, odom (N, 3), status (M, 6: t, detections, matched,
                     new, map cones, ms), landmarks (L, 2), cones (C, 3), and the
                     map after each scan: map_t (K,), map_n (K,), map_xy (sum map_n,
                     2) float32
and with `exit_on_finish` exits.  Stopped early, it writes a partial report.
Topics: /localization/eval/summary (String, 1 Hz), /localization/eval/path_{true,est,dr}
(nav_msgs/Path, 1 Hz) for RViz.
"""

import datetime
import json
import math
import os
import signal
import threading

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from eufs_msgs.msg import ConeArrayWithCovariance
from falcon_msgs.msg import ConeArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from falcon_evaluation import localization as loc


def tf_pose(tf):
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return (tf.transform.translation.x, tf.transform.translation.y, yaw)


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            float(loc.wrap(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class LocalizationEval(Node):
    def __init__(self):
        super().__init__('localization_eval')
        p = self.declare_parameter
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        p('truth_frame', 'base_footprint_gt')
        p('odom_frame', 'odom')
        p('rate_hz', 20.0)
        p('laps', 1)
        p('start_radius_m', 2.5)
        p('min_lap_distance_m', 30.0)
        p('exit_on_finish', False)
        p('output_dir', os.path.expanduser(os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')))
        p('run_name', '')
        p('run_info_json', '{}')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map, self._base = g('map_frame'), g('base_frame')
        self._truth, self._odom = g('truth_frame'), g('odom_frame')
        self._laps_target = int(g('laps'))
        self._start_r = float(g('start_radius_m'))
        self._min_lap = float(g('min_lap_distance_m'))
        self._exit = bool(g('exit_on_finish'))
        self._info = json.loads(g('run_info_json') or '{}')
        name = g('run_name') or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._out = os.path.join(os.path.expanduser(g('output_dir')), name)

        self._t, self._true, self._est, self._dr = [], [], [], []
        self._status = []
        self._landmarks = np.zeros((0, 2))
        self._map_t, self._map_n, self._map_xy = [], [], []
        self._cones = None
        self._map_odom0 = None
        self._last_t = None
        self._start = None
        self._last_pos = None
        self._travelled = 0.0
        self._away = False
        self._laps = 0
        self._done = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_subscription(String, '/localization/status', self._status_cb, 50)
        self.create_subscription(ConeArray, '/localization/landmarks', self._landmarks_cb, 1)
        self.create_subscription(PointCloud2, '/localization/landmark_points', self._points_cb, 50)
        self._track_sub = self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/track', self._track_cb, 1)
        self._summary_pub = self.create_publisher(String, '/localization/eval/summary', 1)
        self._paths = {k: self.create_publisher(Path, f'/localization/eval/path_{k}', 1)
                       for k in ('true', 'est', 'dr')}
        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'Localisation eval: {self._map}->{self._base} against {self._map}->{self._truth}; '
            f'report -> {self._out}')

    # ------------------------------------------------------------------ #
    def _status_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._status.append((d.get('t', 0.0), d.get('detections', 0), d.get('matched', 0),
                             d.get('new', 0), d.get('landmarks', 0), d.get('ms', 0.0)))

    def _landmarks_cb(self, msg):
        self._landmarks = np.array([(c.pose.pose.position.x, c.pose.pose.position.y)
                                    for c in msg.cones], float).reshape(-1, 2)

    def _points_cb(self, msg):
        if self._done:
            return
        n = msg.width * msg.height
        xyz = np.frombuffer(msg.data, dtype='<f4').reshape(-1, msg.point_step // 4)
        self._map_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self._map_n.append(n)
        self._map_xy.append(xyz[:n, :2].astype(np.float32))

    def _maps(self):
        xy = np.concatenate(self._map_xy) if self._map_xy else np.zeros((0, 2), np.float32)
        return np.array(self._map_t, float), np.array(self._map_n, np.int32), xy

    def _track_cb(self, msg):
        rows = []
        for cones, code in ((msg.blue_cones, 1), (msg.yellow_cones, 2), (msg.orange_cones, 3),
                            (msg.big_orange_cones, 4), (msg.unknown_color_cones, 0)):
            rows.extend((c.point.x, c.point.y, code) for c in cones)
        if rows:
            self._cones = np.array(rows, float)
            self.destroy_subscription(self._track_sub)

    def _tick(self):
        if self._done:
            return
        try:
            est_tf = self._tf_buffer.lookup_transform(self._map, self._base, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        stamp = rclpy.time.Time.from_msg(est_tf.header.stamp)
        t = stamp.nanoseconds * 1e-9
        if self._last_t is not None and t <= self._last_t:
            return
        try:
            true_tf = self._tf_buffer.lookup_transform(
                self._map, self._truth, stamp, timeout=rclpy.duration.Duration(seconds=0.05))
            odom_tf = self._tf_buffer.lookup_transform(self._odom, self._base, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        est, truth, odom = tf_pose(est_tf), tf_pose(true_tf), tf_pose(odom_tf)
        if self._map_odom0 is None:
            self._map_odom0 = compose(truth, inverse(odom))
        dr = compose(self._map_odom0, odom)
        self._last_t = t
        self._t.append(t)
        self._true.append(truth)
        self._est.append(est)
        self._dr.append(dr)
        self._update_laps(np.array(truth[:2]))

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
        self.get_logger().info(f'Localisation report written to {path}')
        if self._exit:
            raise SystemExit(0)

    # ------------------------------------------------------------------ #
    def _arrays(self):
        return (np.array(self._t), np.array(self._true).reshape(-1, 3),
                np.array(self._est).reshape(-1, 3), np.array(self._dr).reshape(-1, 3))

    def _publish(self):
        if len(self._t) < 2:
            return
        t, truth, est, dr = self._arrays()
        pos, _, cross, head = loc.pose_errors(truth[-1:], est[-1:])
        dpos = loc.pose_errors(truth[-1:], dr[-1:])[0]
        self._summary_pub.publish(String(data=(
            f'localisation: error {pos[0]:.2f} m (cross {cross[0]:+.2f} m, heading '
            f'{head[0]:+.2f} deg), dead reckoning {dpos[0]:.2f} m; map '
            f'{len(self._landmarks)} cones; lap {self._laps}/{self._laps_target}, '
            f'{self._travelled:.0f} m')))
        idx = loc.by_distance(loc.travelled(truth), 0.5)
        now = self.get_clock().now().to_msg()
        for key, arr in (('true', truth), ('est', est), ('dr', dr)):
            path = Path()
            path.header.stamp, path.header.frame_id = now, self._map
            for x, y, yaw in arr[idx]:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(ps)
            self._paths[key].publish(path)

    def write_report(self):
        if len(self._t) < 2:
            return None
        os.makedirs(self._out, exist_ok=True)
        t, truth, est, dr = self._arrays()
        status = np.array(self._status, float).reshape(-1, 6)
        cones = self._cones if self._cones is not None else np.zeros((0, 3))
        maps = self._maps()
        # The recording first: the summary is computed from it and can be redone later.
        np.savez_compressed(os.path.join(self._out, 'localization.npz'), t=t, truth=truth,
                            est=est, odom=dr, status=status, landmarks=self._landmarks,
                            cones=cones, complete=self._done, laps_completed=self._laps,
                            map_t=maps[0], map_n=maps[1], map_xy=maps[2])
        summary = loc.summarize(t, truth, est, dr, status, self._landmarks,
                                cones if len(cones) else None, maps)
        summary.update(complete=self._done, laps_completed=self._laps,
                       run=self._info, written=datetime.datetime.now().isoformat(timespec='seconds'))
        with open(os.path.join(self._out, 'localization.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(loc.summary_line(summary))
        return self._out


def _quiet_tf_thread(args):
    if args.exc_type.__name__ != 'ExternalShutdownException':
        threading.__excepthook__(args)


def main(args=None):
    rclpy.init(args=args)
    threading.excepthook = _quiet_tf_thread
    node = LocalizationEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if not node._done:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            path = node.write_report()
            if path:
                print(f'[localization_eval] partial report written to {path}', flush=True)
        node._tf_listener.executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
