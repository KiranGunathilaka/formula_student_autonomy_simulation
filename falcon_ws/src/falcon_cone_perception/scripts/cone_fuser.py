#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


class ConeFuser(Node):
    def __init__(self):
        super().__init__("cone_fuser")

        self.latest_camera_cones = None
        self.latest_lidar_cones = None

        # Timestamps of the last received message from each sensor
        self._camera_stamp: Time | None = None
        self._lidar_stamp:  Time | None = None

        # Matching threshold in meters
        self.match_distance_threshold = 0.60

        # If camera cone has no lidar match, keep it only if covariance is
        # small enough (i.e. YOLO was confident enough about position)
        self.max_camera_only_covariance = 0.10

        # -------------------------------------------------------------------
        # FIX: Staleness guard
        #
        # ORIGINAL BUG: the fuser fired on every incoming message and fused
        # against whatever the other sensor last published, with no check on
        # how old that data was.  Camera at 30 Hz + LiDAR at 10 Hz means the
        # fuser emits 20 fused outputs per second using LiDAR data that is up
        # to 100 ms stale.  At vehicle speed this causes spatial mismatches
        # between the two sensors that create spurious detections, which then
        # become spurious landmarks in the map.
        #
        # FIX: only fuse when both sensors have published at least once AND
        # neither message is older than max_sensor_age_s relative to the more
        # recent of the two.  If LiDAR is stale, pass camera-only cones
        # through using the camera-only covariance path (they already exist).
        # If camera is stale, pass LiDAR-only cones through (see below).
        # -------------------------------------------------------------------
        self.max_sensor_age_s = 0.15   # 150 ms — safe for LiDAR at 10 Hz

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

        self.get_logger().info("Cone fuser started.")

    # -----------------------------------------------------------------------
    # Sensor callbacks
    # -----------------------------------------------------------------------

    def camera_callback(self, msg: ConeArrayWithCovariance) -> None:
        self.latest_camera_cones = msg
        self._camera_stamp = Time.from_msg(msg.header.stamp)
        self.try_fuse()

    def lidar_callback(self, msg: ConeArrayWithCovariance) -> None:
        self.latest_lidar_cones = msg
        self._lidar_stamp = Time.from_msg(msg.header.stamp)
        self.try_fuse()

    # -----------------------------------------------------------------------
    # Fusion entry point
    # -----------------------------------------------------------------------

    def try_fuse(self) -> None:
        # Need at least one message from each sensor before fusing
        if self.latest_camera_cones is None or self.latest_lidar_cones is None:
            return

        camera_msg = self.latest_camera_cones
        lidar_msg  = self.latest_lidar_cones

        # -------------------------------------------------------------------
        # FIX: staleness check
        # Compute age of each sensor message relative to the newer one.
        # -------------------------------------------------------------------
        camera_stamp = self._camera_stamp
        lidar_stamp  = self._lidar_stamp

        # Newer of the two is the reference time
        if camera_stamp >= lidar_stamp:
            lidar_age_s  = (camera_stamp - lidar_stamp).nanoseconds * 1e-9
            camera_age_s = 0.0
        else:
            camera_age_s = (lidar_stamp - camera_stamp).nanoseconds * 1e-9
            lidar_age_s  = 0.0

        camera_stale = camera_age_s > self.max_sensor_age_s
        lidar_stale  = lidar_age_s  > self.max_sensor_age_s

        fused_msg = ConeArrayWithCovariance()
        # Use the newer stamp as the fused message stamp
        if camera_stamp >= lidar_stamp:
            fused_msg.header.stamp = camera_msg.header.stamp
        else:
            fused_msg.header.stamp = lidar_msg.header.stamp
        fused_msg.header.frame_id = "base_footprint"

        if camera_stale and lidar_stale:
            # Both sensors are stale — publish nothing, do not spam old data
            self.get_logger().warn(
                "Both camera and LiDAR data are stale — skipping fusion.",
                throttle_duration_sec=1.0,
            )
            return

        if camera_stale:
            # Camera is stale: pass LiDAR-only cones as unknown
            self._passthrough_lidar_only(lidar_msg, fused_msg)
            self.fused_pub.publish(fused_msg)
            return

        # LiDAR candidates (may be stale — handled by camera-only fallback path)
        lidar_candidates = list(lidar_msg.unknown_color_cones) if not lidar_stale else []
        used_lidar: set[int] = set()

        # Fuse each camera color bucket; color is preserved from camera side
        self.fuse_bucket(camera_msg.blue_cones,       lidar_candidates, used_lidar, fused_msg.blue_cones)
        self.fuse_bucket(camera_msg.yellow_cones,     lidar_candidates, used_lidar, fused_msg.yellow_cones)
        self.fuse_bucket(camera_msg.orange_cones,     lidar_candidates, used_lidar, fused_msg.orange_cones)
        self.fuse_bucket(camera_msg.big_orange_cones, lidar_candidates, used_lidar, fused_msg.big_orange_cones)
        self.fuse_bucket(camera_msg.unknown_color_cones, lidar_candidates, used_lidar, fused_msg.unknown_color_cones)

        # -------------------------------------------------------------------
        # FIX: Add unmatched LiDAR cones to the fused output.
        #
        # ORIGINAL BUG: LiDAR cones that had no camera match were silently
        # dropped.  If the camera missed a cone (occlusion, out of FOV, low
        # confidence) the cone disappeared from the fused stream entirely,
        # which eventually caused it to go stale in the map and get pruned.
        #
        # FIX: unmatched LiDAR cones are appended as unknown_color_cones with
        # the LiDAR covariance.  The map builder handles unknown color
        # correctly — it matches anything spatially and does not contribute
        # color votes for a specific color.
        # -------------------------------------------------------------------
        if not lidar_stale:
            for idx, lidar_cone in enumerate(lidar_candidates):
                if idx in used_lidar:
                    continue
                passthrough = ConeWithCovariance()
                passthrough.point.x = float(lidar_cone.point.x)
                passthrough.point.y = float(lidar_cone.point.y)
                passthrough.point.z = 0.0
                # LiDAR-only: use the lidar node's native covariance
                passthrough.covariance = [0.15, 0.0, 0.0, 0.15]
                fused_msg.unknown_color_cones.append(passthrough)

        self.fused_pub.publish(fused_msg)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _passthrough_lidar_only(
        self,
        lidar_msg: ConeArrayWithCovariance,
        fused_msg: ConeArrayWithCovariance,
    ) -> None:
        """Emit all LiDAR cones as unknown when camera is stale."""
        for lidar_cone in lidar_msg.unknown_color_cones:
            cone = ConeWithCovariance()
            cone.point.x = float(lidar_cone.point.x)
            cone.point.y = float(lidar_cone.point.y)
            cone.point.z = 0.0
            cone.covariance = [0.15, 0.0, 0.0, 0.15]
            fused_msg.unknown_color_cones.append(cone)

    def fuse_bucket(
        self,
        camera_bucket,
        lidar_candidates,
        used_lidar: set,
        output_bucket,
    ) -> None:
        for cam_cone in camera_bucket:
            cam_x = cam_cone.point.x
            cam_y = cam_cone.point.y

            best_idx  = -1
            best_dist = float("inf")

            for idx, lidar_cone in enumerate(lidar_candidates):
                if idx in used_lidar:
                    continue
                dx   = lidar_cone.point.x - cam_x
                dy   = lidar_cone.point.y - cam_y
                dist = math.hypot(dx, dy)
                if dist < best_dist:
                    best_dist = dist
                    best_idx  = idx

            matched = best_idx != -1 and best_dist < self.match_distance_threshold

            fused_cone = ConeWithCovariance()

            if matched:
                used_lidar.add(best_idx)
                lidar_cone = lidar_candidates[best_idx]

                lidar_x = lidar_cone.point.x
                lidar_y = lidar_cone.point.y

                # Camera covariance[0] as rough uncertainty proxy
                cam_cov = (
                    float(cam_cone.covariance[0])
                    if len(cam_cone.covariance) >= 4
                    else 0.10
                )

                # Confidence-based weighting
                if cam_cov <= 0.02:
                    w_cam = 0.45
                elif cam_cov <= 0.05:
                    w_cam = 0.35
                else:
                    w_cam = 0.20

                w_lidar = 1.0 - w_cam

                fused_cone.point.x = float(w_cam * cam_x + w_lidar * lidar_x)
                fused_cone.point.y = float(w_cam * cam_y + w_lidar * lidar_y)
                fused_cone.point.z = 0.0

                if cam_cov <= 0.02:
                    fused_cone.covariance = [0.02, 0.0, 0.0, 0.02]
                elif cam_cov <= 0.05:
                    fused_cone.covariance = [0.04, 0.0, 0.0, 0.04]
                else:
                    fused_cone.covariance = [0.06, 0.0, 0.0, 0.06]

            else:
                # No LiDAR match — apply camera-only covariance gate
                cam_cov = (
                    float(cam_cone.covariance[0])
                    if len(cam_cone.covariance) >= 4
                    else 0.10
                )
                if cam_cov > self.max_camera_only_covariance:
                    continue  # too uncertain, drop

                fused_cone.point.x = float(cam_x)
                fused_cone.point.y = float(cam_y)
                fused_cone.point.z = 0.0
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