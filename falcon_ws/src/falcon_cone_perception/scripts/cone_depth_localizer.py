#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from cv_bridge import CvBridge
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


# ---------------------------------------------------------------------------
# Score threshold below which the color label from YOLO is not trusted.
# Detections below this score are emitted as unknown_color_cones so they
# contribute to position fusion without poisoning the color vote in the map.
#
# Why 0.60:
#   - YOLO is launched with threshold=0.25, which is intentionally low to
#     maximise recall.  But scores in [0.25, 0.60) are too uncertain to trust
#     the CLASS label even if the DETECTION itself is real.
#   - At threshold=0.25 a cone seen in poor light or at distance regularly
#     fires with score ~0.30 and the class can flip between blue/yellow.
#   - Positions from low-score detections are still useful (the cone exists),
#     so we keep them as unknown rather than dropping them entirely.
# ---------------------------------------------------------------------------
COLOR_SCORE_THRESHOLD = 0.60


class ConeDepthLocalizer(Node):
    def __init__(self):
        super().__init__("cone_depth_localizer")

        self.bridge = CvBridge()
        self.latest_depth_msg = None
        self.latest_camera_info = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Camera position relative to base_footprint from tf2_echo
        self.cam_tx = -0.080
        self.cam_ty = 0.060
        self.cam_tz = 1.012

        self.create_subscription(
            Image,
            "/zed/depth/image_raw",
            self.depth_callback,
            10,
        )

        self.create_subscription(
            CameraInfo,
            "/zed/camera_info",
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.detections_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/falcon/cone_markers",
            10,
        )
        self.camera_cone_pub = self.create_publisher(
            ConeArrayWithCovariance,
            "/falcon/camera_cones",
            10,
        )

        self.get_logger().info(
            f"Cone depth localizer started. Color score threshold: {COLOR_SCORE_THRESHOLD}"
        )

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def detections_callback(self, msg: DetectionArray) -> None:
        if self.latest_depth_msg is None:
            self.get_logger().warn("No depth image received yet.", throttle_duration_sec=2.0)
            return

        if self.latest_camera_info is None or self.fx is None:
            self.get_logger().warn("No camera info received yet.", throttle_duration_sec=2.0)
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg,
                desired_encoding="32FC1",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        img_h, img_w = depth_image.shape[:2]
        marker_array = MarkerArray()
        cone_msg = ConeArrayWithCovariance()
        cone_msg.header.frame_id = "base_footprint"
        cone_msg.header.stamp = msg.header.stamp

        for det in msg.detections:
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)

            # Sample slightly lower than box centre (base of cone)
            sample_u = int(round(u))
            sample_v = int(round(v + 0.2 * det.bbox.size.y))

            if not (0 <= sample_u < img_w and 0 <= sample_v < img_h):
                self.get_logger().warn(
                    f"Sample pixel out of bounds: ({sample_u}, {sample_v}) "
                    f"for image size ({img_w}, {img_h})"
                )
                continue

            # 7×7 patch around sample point
            patch_radius = 3
            u_min = max(0, sample_u - patch_radius)
            u_max = min(img_w - 1, sample_u + patch_radius)
            v_min = max(0, sample_v - patch_radius)
            v_max = min(img_h - 1, sample_v + patch_radius)

            patch = depth_image[v_min:v_max + 1, u_min:u_max + 1]
            valid_depths = patch[np.isfinite(patch)]
            valid_depths = valid_depths[valid_depths > 0.0]

            if valid_depths.size == 0:
                self.get_logger().warn(
                    f"No valid depth for {det.class_name} near pixel "
                    f"({sample_u}, {sample_v})"
                )
                continue

            z = float(np.median(valid_depths))

            if not math.isfinite(z) or z <= 0.0:
                self.get_logger().warn(
                    f"Invalid median depth for {det.class_name} near pixel "
                    f"({sample_u}, {sample_v}): {z}"
                )
                continue

            # Optical-frame → base_footprint
            x_opt = (u - self.cx) * z / self.fx
            y_opt = (v - self.cy) * z / self.fy
            z_opt = z

            x_base = self.cam_tx + z_opt
            y_base = self.cam_ty - x_opt
            z_base = self.cam_tz - y_opt

            cone = ConeWithCovariance()
            cone.point.x = float(x_base)
            cone.point.y = float(y_base)
            cone.point.z = 0.0

            # Covariance from YOLO detection score
            if det.score > 0.9:
                cone.covariance = [0.02, 0.0, 0.0, 0.02]
            elif det.score > 0.8:
                cone.covariance = [0.05, 0.0, 0.0, 0.05]
            else:
                cone.covariance = [0.10, 0.0, 0.0, 0.10]

            # -------------------------------------------------------------------
            # FIX: Color assignment is gated on detection score.
            #
            # ORIGINAL BUG: color label was always taken from det.class_name
            # regardless of score. With launch threshold=0.25, detections as
            # low as score=0.26 were emitted with a hard color label. YOLO at
            # this score range frequently misclassifies blue vs yellow due to
            # lighting/distance, which fed a wrong color into the map builder
            # every frame.  dominantColor() and the soft color gate in the map
            # builder cannot recover when the wrong color arrives persistently
            # at high vote rate.
            #
            # FIX: below COLOR_SCORE_THRESHOLD the cone is real (position is
            # kept) but the color is not trusted — emitted as unknown so it
            # fuses positionally without accumulating color votes for a wrong
            # color.  Above the threshold YOLO is confident enough that the
            # class label is reliable.
            # -------------------------------------------------------------------
            trusted_color = det.score >= COLOR_SCORE_THRESHOLD

            if trusted_color and det.class_name == "blue_cone":
                cone_msg.blue_cones.append(cone)
                marker_color = (0.0, 0.0, 1.0)
            elif trusted_color and det.class_name == "yellow_cone":
                cone_msg.yellow_cones.append(cone)
                marker_color = (1.0, 1.0, 0.0)
            elif trusted_color and det.class_name == "orange_cone":
                cone_msg.orange_cones.append(cone)
                marker_color = (1.0, 0.5, 0.0)
            elif trusted_color and det.class_name == "large_orange_cone":
                cone_msg.big_orange_cones.append(cone)
                marker_color = (1.0, 0.3, 0.0)
            else:
                # Low confidence OR unrecognised class → unknown color
                cone_msg.unknown_color_cones.append(cone)
                marker_color = (0.7, 0.7, 0.7)

            # Visualisation marker
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = msg.header.stamp
            marker.ns = "cones"
            marker.id = len(marker_array.markers)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x_base
            marker.pose.position.y = y_base
            marker.pose.position.z = z_base
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.a = 1.0
            marker.color.r = marker_color[0]
            marker.color.g = marker_color[1]
            marker.color.b = marker_color[2]

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200_000_000

            marker_array.markers.append(marker)

        self.camera_cone_pub.publish(cone_msg)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDepthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()