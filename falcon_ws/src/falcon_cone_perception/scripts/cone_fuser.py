#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from falcon_msgs.msg import ConeArray as FalconConeArray, Cone as FalconCone


class ConeFuser(Node):
    def __init__(self):
        super().__init__("cone_fuser")

        self.latest_camera_cones = None
        self.latest_lidar_cones = None

        # Matching threshold in meters
        self.match_distance_threshold = 0.60

        # If camera cone has no lidar match, keep it only if covariance is small enough
        # Since camera node publishes confidence as covariance, this becomes a simple gate
        self.max_camera_only_covariance = 0.10

        self.create_subscription(
            ConeArrayWithCovariance,
            "/falcon/camera_cones",
            self.camera_callback,
            10,
        )

        self.create_subscription(
            ConeArrayWithCovariance,
            "/falcon/lidar_cones",
            self.lidar_callback,
            10,
        )

        self.fused_pub = self.create_publisher(
            ConeArrayWithCovariance,
            "/falcon/fused_cones",
            10,
        )

        self.falcon_fused_pub = self.create_publisher(
            FalconConeArray,
            "/perception/cones_fused",
            10,
        )

        self.get_logger().info("Cone fuser started.")

    def camera_callback(self, msg: ConeArrayWithCovariance) -> None:
        self.latest_camera_cones = msg
        self.try_fuse()

    def lidar_callback(self, msg: ConeArrayWithCovariance) -> None:
        self.latest_lidar_cones = msg
        self.try_fuse()

    def try_fuse(self) -> None:
        if self.latest_camera_cones is None or self.latest_lidar_cones is None:
            return

        camera_msg = self.latest_camera_cones
        lidar_msg = self.latest_lidar_cones

        fused_msg = ConeArrayWithCovariance()
        fused_msg.header.stamp = camera_msg.header.stamp
        fused_msg.header.frame_id = "base_footprint"

        # LiDAR cones are expected to be published in unknown_color_cones
        lidar_candidates = list(lidar_msg.unknown_color_cones)
        used_lidar = set()

        # Process each camera color bucket separately so color is preserved
        self.fuse_bucket(
            camera_msg.blue_cones,
            lidar_candidates,
            used_lidar,
            fused_msg.blue_cones,
        )

        self.fuse_bucket(
            camera_msg.yellow_cones,
            lidar_candidates,
            used_lidar,
            fused_msg.yellow_cones,
        )

        self.fuse_bucket(
            camera_msg.orange_cones,
            lidar_candidates,
            used_lidar,
            fused_msg.orange_cones,
        )

        self.fuse_bucket(
            camera_msg.big_orange_cones,
            lidar_candidates,
            used_lidar,
            fused_msg.big_orange_cones,
        )

        self.fuse_bucket(
            camera_msg.unknown_color_cones,
            lidar_candidates,
            used_lidar,
            fused_msg.unknown_color_cones,
        )

        self.fused_pub.publish(fused_msg)
        self.falcon_fused_pub.publish(
            self._to_falcon_msg(fused_msg))

    def _to_falcon_msg(self, eufs_msg: ConeArrayWithCovariance) -> FalconConeArray:
        """Convert eufs_msgs/ConeArrayWithCovariance to falcon_msgs/ConeArray."""
        out = FalconConeArray()
        out.header = eufs_msg.header

        color_buckets = [
            (eufs_msg.blue_cones,          FalconCone.COLOR_BLUE),
            (eufs_msg.yellow_cones,        FalconCone.COLOR_YELLOW),
            (eufs_msg.orange_cones,        FalconCone.COLOR_ORANGE),
            (eufs_msg.big_orange_cones,    FalconCone.COLOR_BIG_ORANGE),
            (eufs_msg.unknown_color_cones, FalconCone.COLOR_UNKNOWN),
        ]

        cone_id = 0
        for bucket, color in color_buckets:
            for ec in bucket:
                fc = FalconCone()
                fc.color = color
                fc.id = cone_id
                fc.confidence = 1.0
                fc.pose.pose.position.x = ec.point.x
                fc.pose.pose.position.y = ec.point.y
                fc.pose.pose.position.z = 0.0
                fc.pose.pose.orientation.w = 1.0
                if len(ec.covariance) >= 4:
                    fc.pose.covariance[0] = ec.covariance[0]
                    fc.pose.covariance[1] = ec.covariance[1]
                    fc.pose.covariance[6] = ec.covariance[2]
                    fc.pose.covariance[7] = ec.covariance[3]
                out.cones.append(fc)
                cone_id += 1

        return out

    def fuse_bucket(
        self,
        camera_bucket,
        lidar_candidates,
        used_lidar,
        output_bucket,
    ) -> None:
        for cam_cone in camera_bucket:
            cam_x = cam_cone.point.x
            cam_y = cam_cone.point.y
            cam_z = cam_cone.point.z

            best_idx = -1
            best_dist = float("inf")

            for idx, lidar_cone in enumerate(lidar_candidates):
                if idx in used_lidar:
                    continue

                dx = lidar_cone.point.x - cam_x
                dy = lidar_cone.point.y - cam_y
                dist = math.hypot(dx, dy)

                if dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            matched = (
                best_idx != -1 and best_dist < self.match_distance_threshold
            )

            fused_cone = ConeWithCovariance()

            if matched:
                used_lidar.add(best_idx)
                lidar_cone = lidar_candidates[best_idx]

                lidar_x = lidar_cone.point.x
                lidar_y = lidar_cone.point.y

                # Camera covariance is [cxx, cxy, cyx, cyy]
                # Use cxx as a rough uncertainty proxy
                cam_cov = float(cam_cone.covariance[0]) if len(cam_cone.covariance) >= 4 else 0.10

                # Confidence-based weighting via covariance:
                # smaller camera covariance -> trust camera more
                # larger camera covariance -> trust lidar more
                if cam_cov <= 0.02:
                    w_cam = 0.45
                elif cam_cov <= 0.05:
                    w_cam = 0.35
                else:
                    w_cam = 0.20

                w_lidar = 1.0 - w_cam

                fused_x = w_cam * cam_x + w_lidar * lidar_x
                fused_y = w_cam * cam_y + w_lidar * lidar_y
                fused_z = 0.0

                fused_cone.point.x = float(fused_x)
                fused_cone.point.y = float(fused_y)
                fused_cone.point.z = float(fused_z)

                # matched cone -> tighter covariance
                if cam_cov <= 0.02:
                    fused_cone.covariance = [0.02, 0.0, 0.0, 0.02]
                elif cam_cov <= 0.05:
                    fused_cone.covariance = [0.04, 0.0, 0.0, 0.04]
                else:
                    fused_cone.covariance = [0.06, 0.0, 0.0, 0.06]

                output_bucket.append(fused_cone)

            else:
                # No LiDAR match
                cam_cov = float(cam_cone.covariance[0]) if len(cam_cone.covariance) >= 4 else 0.10

                # Keep only reasonably confident camera cones
                if cam_cov > self.max_camera_only_covariance:
                    continue

                fused_cone.point.x = float(cam_x)
                fused_cone.point.y = float(cam_y)
                fused_cone.point.z = 0.0

                # camera-only cone gets larger covariance
                fused_cone.covariance = [0.12, 0.0, 0.0, 0.12]

                output_bucket.append(fused_cone)


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