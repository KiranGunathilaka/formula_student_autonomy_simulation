#!/usr/bin/env python3
"""
Join the clouds of a lidar simulated as several Gazebo ray sensors into one cloud.

Gazebo spaces a ray sensor's vertical samples evenly, so falcon_description splits a
lidar with an uneven channel table into one ray sensor per evenly spaced run
(e.g. the Helios 5515 becomes 4).  They share the lidar frame and update rate; this
node waits until every run has published a cloud with the same stamp and publishes
the concatenation on `output_topic`.

Parameters:
  input_topics  list of the per-run topics (falcon_description.urdf_builder.lidar_topics)
  output_topic  default /lidar/points
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LidarMerger(Node):
    def __init__(self):
        super().__init__('lidar_merger')
        self.declare_parameter('input_topics', [''])
        self.declare_parameter('output_topic', '/lidar/points')
        topics = [t for t in self.get_parameter('input_topics').value if t]
        out = self.get_parameter('output_topic').value
        if not topics:
            raise RuntimeError('lidar_merger: input_topics is empty')
        self._n = len(topics)
        self._pending = {}   # stamp (sec, nanosec) -> {index: msg}
        self._pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        for i, t in enumerate(topics):
            self.create_subscription(PointCloud2, t, lambda m, i=i: self._cb(i, m),
                                     qos_profile_sensor_data)
        self.get_logger().info(f'Merging {self._n} clouds {topics} -> {out}')

    def _cb(self, idx, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        slot = self._pending.setdefault(key, {})
        slot[idx] = msg
        if len(slot) < self._n:
            # Keep only the newest few stamps; an older incomplete set will never fill.
            if len(self._pending) > 4:
                for k in sorted(self._pending)[:-4]:
                    del self._pending[k]
            return
        del self._pending[key]
        parts = [slot[i] for i in range(self._n)]
        first = parts[0]
        merged = PointCloud2()
        merged.header = first.header
        merged.height = 1
        merged.fields = first.fields
        merged.is_bigendian = first.is_bigendian
        merged.point_step = first.point_step
        merged.width = sum(p.width * p.height for p in parts)
        merged.row_step = merged.width * merged.point_step
        merged.is_dense = all(p.is_dense for p in parts)
        merged.data = b''.join(bytes(p.data) for p in parts)
        self._pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
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
