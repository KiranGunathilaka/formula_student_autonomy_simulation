#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


class ConeFuser(Node):
    def __init__(self):
        super().__init__("cone_fuser")

        self.latest_camera_poses = None
        self.latest_camera_class_ids = None
        self.latest_camera_scores = None
        self.latest_lidar_poses = None

        self.match_distance_threshold = 0.60
        self.camera_only_min_confidence = 0.75

        self.create_subscription(
            PoseArray,
            "/falcon/camera_cone_poses",
            self.camera_pose_callback,
            10,
        )

        self.create_subscription(
            Int32MultiArray,
            "/falcon/camera_cone_class_ids",
            self.camera_class_callback,
            10,
        )

        self.create_subscription(
            Float32MultiArray,
            "/falcon/camera_cone_scores",
            self.camera_score_callback,
            10,
        )

        self.create_subscription(
            PoseArray,
            "/falcon/lidar_cluster_poses",
            self.lidar_pose_callback,
            10,
        )

        self.fused_pub = self.create_publisher(
            ConeArrayWithCovariance,
            "/falcon/fused_cones",
            10,
        )

        self.get_logger().info("Cone fuser started.")

    def camera_pose_callback(self, msg):
        self.latest_camera_poses = msg
        self.try_fuse()

    def camera_class_callback(self, msg):
        self.latest_camera_class_ids = msg
        self.try_fuse()

    def camera_score_callback(self, msg):
        self.latest_camera_scores = msg
        self.try_fuse()

    def lidar_pose_callback(self, msg):
        self.latest_lidar_poses = msg
        self.try_fuse()

    def try_fuse(self):
        if self.latest_camera_poses is None:
            print("No camera poses yet.")
            return
        if self.latest_camera_class_ids is None:
            print("No camera class IDs yet.")
            return
        if self.latest_camera_scores is None:
            print("No camera scores yet.")
            return
        if self.latest_lidar_poses is None:
            print("No LiDAR poses yet.")
            return

        camera_poses = self.latest_camera_poses.poses
        lidar_poses = self.latest_lidar_poses.poses
        class_ids = self.latest_camera_class_ids.data
        scores = self.latest_camera_scores.data

        if not (len(camera_poses) == len(class_ids) == len(scores)):
            self.get_logger().warn("Camera pose/class/score lengths do not match.")
            return

        fused_msg = ConeArrayWithCovariance()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "base_footprint"

        used_lidar = set()

        for i, cam_pose in enumerate(camera_poses):
            cam_x = cam_pose.position.x
            cam_y = cam_pose.position.y
            cam_z = cam_pose.position.z

            class_id = int(class_ids[i])
            score = float(scores[i])

            best_idx = -1
            best_dist = float("inf")

            for j, lidar_pose in enumerate(lidar_poses):
                if j in used_lidar:
                    continue

                dx = lidar_pose.position.x - cam_x
                dy = lidar_pose.position.y - cam_y
                dist = math.hypot(dx, dy)

                if dist < best_dist:
                    best_dist = dist
                    best_idx = j

            matched = (best_idx != -1 and best_dist < self.match_distance_threshold)

            if matched:
                used_lidar.add(best_idx)

                lidar_x = lidar_poses[best_idx].position.x
                lidar_y = lidar_poses[best_idx].position.y

                # confidence-based blending
                # camera contributes more when confidence is high
                w_cam = 0.2 + 0.3 * max(0.0, min(score, 1.0))  # 0.2 to 0.5
                w_lidar = 1.0 - w_cam

                fused_x = w_cam * cam_x + w_lidar * lidar_x
                fused_y = w_cam * cam_y + w_lidar * lidar_y
                fused_z = 0.0

                if score >= 0.90:
                    covariance = [0.02, 0.0, 0.0, 0.02]
                elif score >= 0.80:
                    covariance = [0.04, 0.0, 0.0, 0.04]
                else:
                    covariance = [0.06, 0.0, 0.0, 0.06]

            else:
                if score < self.camera_only_min_confidence:
                    continue

                fused_x = cam_x
                fused_y = cam_y
                fused_z = 0.0
                covariance = [0.16, 0.0, 0.0, 0.16]

            cone = ConeWithCovariance()
            cone.point.x = float(fused_x)
            cone.point.y = float(fused_y)
            cone.point.z = float(fused_z)
            cone.covariance = [float(c) for c in covariance]

            # class mapping from your YOLO training
            # 0 -> blue_cone
            # 1 -> yellow_cone
            # 2 -> large_orange_cone
            # 3 -> orange_cone
            # 4 -> unknown_cone
            if class_id == 0:
                fused_msg.blue_cones.append(cone)
            elif class_id == 1:
                fused_msg.yellow_cones.append(cone)
            elif class_id == 2:
                fused_msg.big_orange_cones.append(cone)
            elif class_id == 3:
                fused_msg.orange_cones.append(cone)
            else:
                fused_msg.unknown_colour_cones.append(cone)

        self.fused_pub.publish(fused_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()