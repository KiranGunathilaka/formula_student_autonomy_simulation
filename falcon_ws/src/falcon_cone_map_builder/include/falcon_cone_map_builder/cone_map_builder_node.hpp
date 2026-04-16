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

#include <std_msgs/msg/color_rgba.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
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
  explicit ConeMapBuilderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // -------------------------------------------------------------------------
  // Internal types
  // -------------------------------------------------------------------------
  struct Observation
  {
    Eigen::Vector2d position;
    Eigen::Matrix2d covariance;
    uint8_t         color;
  };

  // -------------------------------------------------------------------------
  // Callbacks
  // -------------------------------------------------------------------------
  void conesCallback(
    const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);

  void publishTimerCallback();

  // -------------------------------------------------------------------------
  // TF2 helpers
  // -------------------------------------------------------------------------
  bool transformCones(
    const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr & msg,
    std::vector<Observation> & observations);

  void extractCones(
    const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones,
    uint8_t color,
    const geometry_msgs::msg::TransformStamped & tf,
    std::vector<Observation> & observations);

  // -------------------------------------------------------------------------
  // Data association and fusion
  // -------------------------------------------------------------------------
  void processObservations(
    const std::vector<Observation> & observations,
    double stamp_sec);

  // Primary gate: Mahalanobis distance with soft color penalty.
  int findNearestLandmark(
    const Observation & obs, double & best_distance) const;

  // Fallback gate: color-blind Euclidean proximity used before landmark
  // creation to prevent duplicates caused by color-penalised Mahalanobis
  // distances exceeding the threshold.
  int findNearestLandmarkEuclidean(const Observation & obs) const;

  void fuseLandmark(
    Landmark & lm, const Observation & obs, double stamp_sec);

  Landmark createLandmark(
    const Observation & obs, double stamp_sec);

  // -------------------------------------------------------------------------
  // Pruning
  // -------------------------------------------------------------------------

  // Time-based staleness pruning (existing): removes landmarks not observed
  // within stale_timeout_s_. No-op when stale_timeout_s_ <= 0.
  void pruneStale(double now_sec);

  // ---------------------------------------------------------------------------
  // Relative-count pruning (NEW):
  //
  // Removes landmarks whose observation_count is much lower than that of a
  // nearby neighbor, indicating they are false/duplicate landmarks rather than
  // real cones.
  //
  // Rationale: after the association fixes, real cones accumulate observations
  // on every vehicle pass (hundreds of counts over a run) while false/duplicate
  // landmarks stagnate at low counts (< ~20) because nearly all detections are
  // now correctly routed to the dominant neighbor. The count gap is reliable
  // and robust enough to use as a pruning signal.
  //
  // A landmark is pruned when ALL of the following hold:
  //   1. Its observation_count >= relative_prune_min_observations_
  //      (protects genuinely new landmarks that simply haven't been seen yet)
  //   2. There exists a neighbor within relative_prune_radius_ whose
  //      observation_count > (this landmark's count / relative_prune_ratio_)
  //      i.e. the neighbor has proportionally far more observations
  //
  // Called from conesCallback after every observation batch.
  // ---------------------------------------------------------------------------
  void pruneRelative();

  // -------------------------------------------------------------------------
  // Publishing
  // -------------------------------------------------------------------------
  void publishMap(const rclcpp::Time & stamp);
  void publishMarkers(const rclcpp::Time & stamp);

  static std_msgs::msg::ColorRGBA colorForCone(uint8_t color);

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  std::string input_topic_;
  std::string output_map_topic_;
  std::string output_marker_topic_;
  std::string map_frame_;

  double   mahalanobis_threshold_;
  double   publish_rate_hz_;
  double   stale_timeout_s_;
  uint32_t confidence_saturation_;
  double   cone_marker_height_;
  double   cone_marker_radius_;
  bool     color_gated_association_;
  double   tf_timeout_s_;
  int      ellipse_segments_;

  // Soft color gate
  double color_lock_threshold_;
  double color_mismatch_penalty_;

  // Euclidean fallback (duplicate prevention)
  double euclidean_fallback_radius_;

  // Relative-count pruning
  bool     relative_prune_enabled_;
  double   relative_prune_radius_;
  double   relative_prune_ratio_;
  uint32_t relative_prune_min_observations_;

  // -------------------------------------------------------------------------
  // TF2
  // -------------------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // -------------------------------------------------------------------------
  // ROS interfaces
  // -------------------------------------------------------------------------
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cones_sub_;
  rclcpp::Publisher<falcon_msgs::msg::ConeArray>::SharedPtr                map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr       marker_pub_;
  rclcpp::TimerBase::SharedPtr                                             publish_timer_;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  std::vector<Landmark> landmarks_;
  uint32_t              next_id_;
  std::set<uint32_t>    prev_landmark_ids_;
  std::mutex            map_mutex_;
};

}  // namespace falcon_cone_map_builder