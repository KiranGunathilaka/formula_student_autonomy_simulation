"""
Cone Bridge Node
----------------
Subscribes to /cones (eufs_msgs/ConeArrayWithCovariance) from the EUFS simulator
and republishes as /perception/cones_raw (falcon_msgs/ConeArray).

Cones are forwarded in the same frame they arrive in (base_footprint — vehicle
frame).  No TF transform is performed, keeping this node dependency-free and
robust to TF tree variations across simulator versions.

Topic flow:
  EUFS sim → /cones (eufs_msgs, base_footprint frame)
           → [cone_bridge_node]
           → /perception/cones_raw (falcon_msgs, base_footprint frame)
           → cone_fusion_node
           → /perception/cones_fused (falcon_msgs, base_footprint frame)
"""

import rclpy
from rclpy.node import Node

from eufs_msgs.msg import ConeArrayWithCovariance
from falcon_msgs.msg import ConeArray, Cone


class ConeBridgeNode(Node):

    def __init__(self):
        super().__init__('cone_bridge_node')

        self.declare_parameter('input_topic', '/cones')
        self.declare_parameter('output_topic', '/perception/cones_raw')

        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value

        self._sub = self.create_subscription(
            ConeArrayWithCovariance,
            self._input_topic,
            self._callback,
            10,
        )
        self._pub = self.create_publisher(ConeArray, self._output_topic, 10)

        self.get_logger().info(
            f'Cone bridge ready: {self._input_topic} → {self._output_topic}'
        )

    def _callback(self, msg: ConeArrayWithCovariance):
        out = ConeArray()
        out.header = msg.header   # keep original frame (base_footprint)

        # Map of eufs cone lists → falcon color constants
        color_groups = [
            (msg.blue_cones,          Cone.COLOR_BLUE),
            (msg.yellow_cones,        Cone.COLOR_YELLOW),
            (msg.orange_cones,        Cone.COLOR_ORANGE),
            (msg.big_orange_cones,    Cone.COLOR_BIG_ORANGE),
            (msg.unknown_color_cones, Cone.COLOR_UNKNOWN),
        ]

        cone_id = 0
        for eufs_list, color in color_groups:
            for eufs_cone in eufs_list:
                c = Cone()
                c.color = color
                c.id = cone_id
                c.confidence = 1.0
                c.pose.pose.position.x = eufs_cone.point.x
                c.pose.pose.position.y = eufs_cone.point.y
                c.pose.pose.position.z = 0.0
                c.pose.pose.orientation.w = 1.0
                # Propagate covariance from eufs (4-element: xx, xy, yx, yy)
                cov = eufs_cone.covariance
                if len(cov) >= 4:
                    c.pose.covariance[0] = cov[0]   # xx
                    c.pose.covariance[1] = cov[1]   # xy
                    c.pose.covariance[6] = cov[2]   # yx
                    c.pose.covariance[7] = cov[3]   # yy

                out.cones.append(c)
                cone_id += 1

        self._pub.publish(out)
        self.get_logger().debug(
            f'Published {len(out.cones)} cones', throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ConeBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()
