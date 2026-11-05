"""
perception_check: how far the perceived cones are from the real ones, while the sim runs.

  ros2 run falcon_evaluation perception_check            # 60 s
  ros2 run falcon_evaluation perception_check --seconds 120

Every cone published on /falcon/lidar_cones, /falcon/camera_cones and
/falcon/fused_cones is moved into the map frame (TF at the message stamp) and matched
to the nearest cone of the true track (/ground_truth/track).  Prints, per stage, the
median and 90th-percentile distance, the same within 15 m of the car, and the share of
cones within 15 m that are more than 0.5 m from any real cone (false or badly placed).
Also: of the real cones within 10 m ahead (+-55 deg, the camera's half view), the
share with a fused cone within 0.5 m.  A message waits until the car's pose at its
stamp is in TF; one still without it after 1 s is skipped and counted.
"""

import argparse
import collections
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from eufs_msgs.msg import ConeArrayWithCovariance

TOPICS = (('lidar', '/falcon/lidar_cones'), ('camera', '/falcon/camera_cones'),
          ('fused', '/falcon/fused_cones'))


def cone_xy(msg):
    pts = []
    for cones in (msg.blue_cones, msg.yellow_cones, msg.orange_cones, msg.big_orange_cones,
                  msg.unknown_color_cones):
        pts += [(c.point.x, c.point.y) for c in cones]
    return np.array(pts, dtype=float).reshape(-1, 2)


class PerceptionCheck:
    def __init__(self, node):
        self.node = node
        self.tf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.tf, node, spin_thread=True)
        self.track = None
        self.err = {k: [] for k, _ in TOPICS}       # (distance to real cone, range from car)
        self.msgs = {k: [0, 0, 0] for k, _ in TOPICS}  # messages, cones, skipped (no TF)
        self.recall = []
        node.create_subscription(ConeArrayWithCovariance, '/ground_truth/track', self._track, 1)
        # Messages wait here until the pose at their stamp is in TF (it trails under load).
        self.pending = collections.deque(maxlen=200)
        for name, topic in TOPICS:
            node.create_subscription(ConeArrayWithCovariance, topic,
                                     lambda m, n=name: self.pending.append((n, m)), 10)
        node.create_timer(0.05, self.drain)

    def _track(self, msg):
        if self.track is None:
            self.track = cone_xy(msg)

    def drain(self):
        now = self.node.get_clock().now()
        keep = collections.deque(maxlen=self.pending.maxlen)
        while self.pending:
            name, msg = self.pending.popleft()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if self.tf.can_transform('map', msg.header.frame_id, stamp):
                self._cones(name, msg, self.tf.lookup_transform('map', msg.header.frame_id, stamp))
            elif (now - stamp).nanoseconds > 1e9:
                self.msgs[name][2] += 1
            else:
                keep.append((name, msg))
        self.pending = keep

    @staticmethod
    def _pose(t):
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return math.cos(yaw), math.sin(yaw), t.transform.translation.x, t.transform.translation.y

    def _cones(self, name, msg, tf):
        T = self.track
        if T is None:
            return
        c, s, tx, ty = self._pose(tf)
        P = cone_xy(msg)
        self.msgs[name][0] += 1
        self.msgs[name][1] += len(P)
        if len(P):
            Q = np.stack([c * P[:, 0] - s * P[:, 1] + tx, s * P[:, 0] + c * P[:, 1] + ty], 1)
            d = np.hypot(Q[:, None, 0] - T[None, :, 0], Q[:, None, 1] - T[None, :, 1]).min(axis=1)
            self.err[name] += list(zip(d, np.hypot(P[:, 0], P[:, 1])))
        if name == 'fused':
            dx, dy = T[:, 0] - tx, T[:, 1] - ty
            bx, by = c * dx + s * dy, -s * dx + c * dy
            ahead = (np.hypot(bx, by) < 10) & (bx > 0) & \
                (np.abs(np.degrees(np.arctan2(by, bx))) < 55)
            if ahead.any():
                if len(P):
                    dd = np.hypot(bx[ahead][:, None] - P[None, :, 0],
                                  by[ahead][:, None] - P[None, :, 1]).min(axis=1)
                    self.recall.append(float(np.mean(dd < 0.5)))
                else:
                    self.recall.append(0.0)

    def report(self):
        lines = []
        for name, _ in TOPICS:
            msgs, cones, skipped = self.msgs[name]
            if not self.err[name]:
                lines.append(f'{name:6s}: no cones ({msgs} messages, {skipped} without TF)')
                continue
            e = np.array(self.err[name])
            near = e[e[:, 1] < 15]
            far_off = 100 * np.mean(near[:, 0] > 0.5) if len(near) else float('nan')
            lines.append(
                f'{name:6s}: {msgs} msgs, {cones / max(msgs, 1):.1f} cones/msg | error to nearest GT '
                f'cone: median {np.median(e[:, 0]):.3f} m, p90 {np.percentile(e[:, 0], 90):.3f} m | '
                f'within 15 m: median {np.median(near[:, 0]) if len(near) else float("nan"):.3f} m | '
                f'within 15 m: {len(near)} detections, {far_off:.1f}% more than 0.5 m from any cone | '
                f'{skipped} msgs without TF')
        if self.recall:
            lines.append(f'fused recall, track cones within 10 m ahead: mean '
                         f'{np.mean(self.recall):.3f} over {len(self.recall)} msgs')
        return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=60.0, help='wall time to listen')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'] + ros_args)
    node = rclpy.create_node('perception_check')
    check = PerceptionCheck(node)
    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print(check.report(), flush=True)
    # The TF listener's thread would keep the process alive.
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
