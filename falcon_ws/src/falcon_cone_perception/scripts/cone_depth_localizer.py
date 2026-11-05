#!/usr/bin/env python3

import math
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
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

        # Camera pose in base_footprint, from TF (the URDF built from the vehicle
        # profile), so it follows the camera height and pitch of whichever car is
        # loaded.  It used to be hard-coded to the EUFS car's ZED with no pitch.
        self.base_frame = "base_footprint"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cam_R = None
        self.cam_t = None

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
        if self.cam_R is None:
            self.lookup_camera_pose(msg.header.frame_id)

    def lookup_camera_pose(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"No TF {frame} -> {self.base_frame}: {e}",
                                   throttle_duration_sec=2.0)
            return
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.cam_R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        t = tf.transform.translation
        self.cam_t = np.array([t.x, t.y, t.z])
        self.get_logger().info(
            f"Camera {frame} at x={t.x:.3f} y={t.y:.3f} z={t.z:.3f} in {self.base_frame}")

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

        if self.cam_R is None:
            self.get_logger().warn("No camera pose from TF yet.", throttle_duration_sec=2.0)
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

            # The Gazebo camera looks along +x of its frame: optical (right, down,
            # forward) -> frame (forward, left, up), then the mount (incl. pitch).
            p_base = self.cam_R @ np.array([z_opt, -x_opt, -y_opt]) + self.cam_t
            x_base, y_base, z_base = (float(v) for v in p_base)

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