#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from falcon_msgs.msg import Cone, ConeArray
from geometry_msgs.msg import PoseWithCovariance, Pose, Vector3
from std_msgs.msg import Header


def make_cone(cone_id, color, x, y, z, confidence=0.9):
    cone = Cone()
    cone.id = cone_id
    cone.color = color
    cone.pose = PoseWithCovariance()
    cone.pose.pose = Pose()
    cone.pose.pose.position.x = float(x)
    cone.pose.pose.position.y = float(y)
    cone.pose.pose.position.z = float(z)
    cone.pose.pose.orientation.w = 1.0
    cone.pose.pose.orientation.x = 0.0
    cone.pose.pose.orientation.y = 0.0
    cone.pose.pose.orientation.z = 0.0
    cov = 0.01
    cone.pose.covariance = [
        cov, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, cov, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, cov, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, cov, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, cov, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, cov,
    ]
    cone.confidence = confidence
    cone.size = Vector3()
    cone.size.x = 0.23
    cone.size.y = 0.23
    cone.size.z = 0.3
    return cone


FUSED_CONES = [
    (1, Cone.COLOR_BLUE, 5.0, -2.0, 0.0),
    (2, Cone.COLOR_BLUE, 8.0, -2.0, 0.0),
    (3, Cone.COLOR_YELLOW, 5.0, 2.0, 0.0),
    (4, Cone.COLOR_YELLOW, 8.0, 2.0, 0.0),
    (5, Cone.COLOR_BLUE, 11.0, -2.0, 0.0),
    (6, Cone.COLOR_YELLOW, 11.0, 2.0, 0.0),
    (7, Cone.COLOR_ORANGE, 14.0, 0.0, 0.0),
    (8, Cone.COLOR_BLUE, 17.0, -2.0, 0.0),
    (9, Cone.COLOR_YELLOW, 17.0, 2.0, 0.0),
]

MAP_CONES = [
    (101, Cone.COLOR_BLUE, 5.0, -2.0, 0.0),
    (102, Cone.COLOR_BLUE, 8.0, -2.0, 0.0),
    (103, Cone.COLOR_YELLOW, 5.0, 2.0, 0.0),
    (104, Cone.COLOR_YELLOW, 8.0, 2.0, 0.0),
    (105, Cone.COLOR_BLUE, 11.0, -2.0, 0.0),
    (106, Cone.COLOR_YELLOW, 11.0, 2.0, 0.0),
    (107, Cone.COLOR_ORANGE, 14.0, 0.0, 0.0),
    (108, Cone.COLOR_BLUE, 17.0, -2.0, 0.0),
    (109, Cone.COLOR_YELLOW, 17.0, 2.0, 0.0),
]


class FalconSimNode(Node):
    def __init__(self):
        super().__init__('falcon_sim_node')
        self.declare_parameter('enable_fused', True)
        self.declare_parameter('enable_map', True)
        self.declare_parameter('fused_topic', '/perception/cones_fused')
        self.declare_parameter('map_topic', '/map/cones_map')
        self.declare_parameter('fused_rate_hz', 10.0)
        self.declare_parameter('map_rate_hz', 1.0)

        enable_fused = self.get_parameter('enable_fused').get_parameter_value().bool_value
        enable_map = self.get_parameter('enable_map').get_parameter_value().bool_value
        fused_topic = self.get_parameter('fused_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        fused_rate_hz = self.get_parameter('fused_rate_hz').get_parameter_value().double_value
        map_rate_hz = self.get_parameter('map_rate_hz').get_parameter_value().double_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._fused_pub = None
        self._map_pub = None
        self._fused_timer = None
        self._map_timer = None

        if enable_fused:
            self._fused_pub = self.create_publisher(ConeArray, fused_topic, qos)
            period = 1.0 / fused_rate_hz if fused_rate_hz > 0 else 1.0
            self._fused_timer = self.create_timer(period, self._publish_fused)

        if enable_map:
            self._map_pub = self.create_publisher(ConeArray, map_topic, qos)
            period = 1.0 / map_rate_hz if map_rate_hz > 0 else 1.0
            self._map_timer = self.create_timer(period, self._publish_map)

    def _publish_fused(self):
        if self._fused_pub is None:
            return
        msg = ConeArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.cones = [make_cone(cid, c, x, y, z) for cid, c, x, y, z in FUSED_CONES]
        self._fused_pub.publish(msg)

    def _publish_map(self):
        if self._map_pub is None:
            return
        msg = ConeArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.cones = [make_cone(cid, c, x, y, z) for cid, c, x, y, z in MAP_CONES]
        self._map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FalconSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
