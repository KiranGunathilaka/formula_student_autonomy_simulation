#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


class LidarConeDetector(Node):
    def __init__(self):
        super().__init__("lidar_cone_detector")

        # LiDAR mount position in base_footprint
        self.lidar_x = 1.683
        self.lidar_y = 0.0
        self.lidar_z = 0.0
        
        self.min_useful_range = 0.25
        self.max_useful_range = 9.0

        self.car_x_min = -0.8
        self.car_x_max = 1.78
        self.car_y_min = -0.85
        self.car_y_max = 0.85

        # Clustering threshold
        self.cluster_distance_threshold = 0.10

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/gazebo_scan",
            self.scan_callback,
            10,
        )

        self.filtered_pub = self.create_publisher(
            MarkerArray,
            "/falcon/lidar_filtered_markers",
            10,
        )

        self.cluster_pub = self.create_publisher(
            MarkerArray,
            "/falcon/lidar_cluster_markers",
            10,
        )

        self.lidar_cone_pub = self.create_publisher(
            ConeArrayWithCovariance,
            "/falcon/lidar_cones",
            10,
        )

        self.get_logger().info("LiDAR cone detector and clustering node started")

    def scan_callback(self, msg: LaserScan) -> None:
        filtered_points = []

        angle = msg.angle_min

        for r in msg.ranges:
            if not math.isfinite(r):
                angle += msg.angle_increment
                continue

            if r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            if r < self.min_useful_range or r > self.max_useful_range:
                angle += msg.angle_increment
                continue

            x_lidar = r * math.cos(angle)
            y_lidar = r * math.sin(angle)

            x_base = self.lidar_x + x_lidar
            y_base = self.lidar_y + y_lidar

            if (
                self.car_x_min <= x_base <= self.car_x_max
                and self.car_y_min <= y_base <= self.car_y_max
            ):
                angle += msg.angle_increment
                continue

            filtered_points.append((x_base, y_base))
            angle += msg.angle_increment

        self.publish_filtered_points(msg, filtered_points)

        clusters = self.cluster_points(filtered_points)
        self.publish_cluster_centroids(msg, clusters)

    def cluster_points(self, points):
        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev_x, prev_y = points[i - 1]
            curr_x, curr_y = points[i]

            dist = math.hypot(curr_x - prev_x, curr_y - prev_y)

            if dist < self.cluster_distance_threshold:
                current_cluster.append(points[i])
            else:
                clusters.append(current_cluster)
                current_cluster = [points[i]]

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def publish_filtered_points(self, msg: LaserScan, points):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = msg.header.stamp

            marker.ns = "lidar_filtered_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 150000000

            marker_array.markers.append(marker)

        self.filtered_pub.publish(marker_array)

    def publish_cluster_centroids(self, msg: LaserScan, clusters):
        marker_array = MarkerArray()
        cone_msg = ConeArrayWithCovariance()
        cone_msg.header.frame_id = "base_footprint"
        cone_msg.header.stamp = msg.header.stamp

        for i, cluster in enumerate(clusters):
            if len(cluster) == 0:
                continue

            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            cone = ConeWithCovariance()
            cone.point.x = float(cx)
            cone.point.y = float(cy)
            cone.point.z = 0.0

            # lidar uncertainty (bigger than camera)
            cone.covariance = [0.15, 0.0, 0.0, 0.15]

            cone_msg.unknown_color_cones.append(cone)

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = msg.header.stamp

            marker.ns = "lidar_cluster_centroids"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.12
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.14

            # simple repeating colors
            colors = [
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
                (1.0, 1.0, 0.0),
                (1.0, 0.0, 1.0),
                (0.0, 1.0, 1.0),
            ]

            r, g, b = colors[i % len(colors)]

            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 150000000

            marker_array.markers.append(marker)

        self.lidar_cone_pub.publish(cone_msg)
        self.cluster_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()