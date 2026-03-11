#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <falcon_msgs/msg/cone_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "falcon_cone_map_builder/landmark.hpp"

namespace falcon_cone_map_builder
{

class ConeMapBuilderNode : public rclcpp::Node
{
public:
  explicit ConeMapBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Observation
  {
    Eigen::Vector2d position;
    Eigen::Matrix2d covariance;
    uint8_t color;
  };

  // Subscription callback
  void conesCallback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);

  // Timer-driven publish
  void publishTimerCallback();

  // TF2 transform all cones in the message into map frame
  bool transformCones(
    const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr & msg,
    std::vector<Observation> & observations);

  // Extract and transform a single color array
  void extractCones(
    const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones,
    uint8_t color,
    const geometry_msgs::msg::TransformStamped & tf,
    std::vector<Observation> & observations);

  // Associate observations with existing landmarks and fuse
  void processObservations(
    const std::vector<Observation> & observations,
    double stamp_sec);

  // Return index of nearest landmark under Mahalanobis gate, or -1
  int findNearestLandmark(const Observation & obs, double & best_distance) const;

  // Kalman-style covariance-weighted fusion into existing landmark
  void fuseLandmark(Landmark & lm, const Observation & obs, double stamp_sec);

  // Initialize a new landmark from an unmatched observation
  Landmark createLandmark(const Observation & obs, double stamp_sec);

  // Remove landmarks not observed within the stale timeout
  void pruneStale(double now_sec);

  // Publish the global cone map as falcon_msgs::msg::ConeArray
  void publishMap(const rclcpp::Time & stamp);

  // Publish RViz visualization markers (cones, IDs, covariance ellipses)
  void publishMarkers(const rclcpp::Time & stamp);

  // Map cone color enum to an RGBA value for RViz
  static std_msgs::msg::ColorRGBA colorForCone(uint8_t color);

  // --- Parameters ---
  std::string input_topic_;
  std::string output_map_topic_;
  std::string output_marker_topic_;
  std::string map_frame_;
  double mahalanobis_threshold_;
  double publish_rate_hz_;
  double stale_timeout_s_;
  uint32_t confidence_saturation_;
  double cone_marker_height_;
  double cone_marker_radius_;
  bool color_gated_association_;
  double tf_timeout_s_;
  int ellipse_segments_;

  // --- TF2 ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- ROS interfaces ---
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cones_sub_;
  rclcpp::Publisher<falcon_msgs::msg::ConeArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // --- State ---
  std::vector<Landmark> landmarks_;
  uint32_t next_id_;
  std::set<uint32_t> prev_landmark_ids_;
  std::mutex map_mutex_;
};

}  // namespace falcon_cone_map_builder
