#include "falcon_cone_map_builder/cone_map_builder_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace falcon_cone_map_builder
{

// =============================================================================
// Constructor
// =============================================================================

ConeMapBuilderNode::ConeMapBuilderNode(const rclcpp::NodeOptions & options)
: Node("falcon_cone_map_builder_node", options), next_id_(0)
{
  // ---------------------------------------------------------------------------
  // Standard parameters
  // ---------------------------------------------------------------------------
  declare_parameter<std::string>("input_topic",         "/cones");
  declare_parameter<std::string>("output_map_topic",    "/map/cone_map");
  declare_parameter<std::string>("output_marker_topic", "/map/cone_markers");
  declare_parameter<std::string>("map_frame",           "map");
  declare_parameter<double>("mahalanobis_threshold",    3.0);
  declare_parameter<double>("publish_rate_hz",          10.0);
  declare_parameter<double>("stale_timeout_s",          0.0);
  declare_parameter<int>("confidence_saturation",       15);
  declare_parameter<double>("cone_marker_height",       0.3);
  declare_parameter<double>("cone_marker_radius",       0.1);
  declare_parameter<bool>("color_gated_association",    true);
  declare_parameter<double>("tf_timeout_s",             0.1);
  declare_parameter<int>("ellipse_segments",            32);

  // ---------------------------------------------------------------------------
  // Soft color-gate parameters
  // ---------------------------------------------------------------------------
  declare_parameter<double>("color_lock_threshold",   0.8);
  declare_parameter<double>("color_mismatch_penalty", 2.0);

  // ---------------------------------------------------------------------------
  // Euclidean fallback (duplicate prevention)
  // ---------------------------------------------------------------------------
  declare_parameter<double>("euclidean_fallback_radius", 1.0);

  // ---------------------------------------------------------------------------
  // Relative-count pruning parameters
  //
  //   relative_prune_enabled (bool, default: true)
  //     Master switch. Disable without removing other parameters.
  //
  //   relative_prune_radius (double, meters, default: 2.0)
  //     Euclidean search radius for finding dominating neighbors.
  //     Should be large enough to always capture the real cone a false
  //     duplicate sits beside, but small enough not to reach genuinely
  //     separate real cones on the far side of the track.
  //     Recommended: 1.5 – 2.5 m for a Formula Student track.
  //
  //   relative_prune_ratio (double, 0.0 – 1.0, default: 0.1)
  //     A landmark is pruned if its count < (neighbor_count * ratio).
  //     Equivalently: the neighbor must have at least (1/ratio)x more
  //     observations. Default 0.1 means a 10:1 ratio is required.
  //     Lower  → more aggressive pruning (e.g. 0.05 = 20:1 required)
  //     Higher → more conservative pruning (e.g. 0.2 = 5:1 required)
  //
  //   relative_prune_min_observations (int, default: 30)
  //     A landmark is NEVER a prune candidate until its own
  //     observation_count reaches this floor, regardless of neighbor counts.
  //     This prevents newly-created real landmarks from being pruned before
  //     they have had a chance to accumulate observations.
  //     Rule of thumb: perception_rate_hz * grace_period_seconds.
  //     At 10 Hz with a 3 s grace period → 30.
  // ---------------------------------------------------------------------------
  declare_parameter<bool>("relative_prune_enabled",            true);
  declare_parameter<double>("relative_prune_radius",           20.0);
  declare_parameter<double>("relative_prune_ratio",            0.2);
  declare_parameter<int>("relative_prune_min_observations",    10);

  // Read standard parameters
  input_topic_           = get_parameter("input_topic").as_string();
  output_map_topic_      = get_parameter("output_map_topic").as_string();
  output_marker_topic_   = get_parameter("output_marker_topic").as_string();
  map_frame_             = get_parameter("map_frame").as_string();
  mahalanobis_threshold_ = get_parameter("mahalanobis_threshold").as_double();
  publish_rate_hz_       = get_parameter("publish_rate_hz").as_double();
  stale_timeout_s_       = get_parameter("stale_timeout_s").as_double();
  confidence_saturation_ =
    static_cast<uint32_t>(get_parameter("confidence_saturation").as_int());
  cone_marker_height_      = get_parameter("cone_marker_height").as_double();
  cone_marker_radius_      = get_parameter("cone_marker_radius").as_double();
  color_gated_association_ = get_parameter("color_gated_association").as_bool();
  tf_timeout_s_            = get_parameter("tf_timeout_s").as_double();
  ellipse_segments_        = get_parameter("ellipse_segments").as_int();

  // Read color-gate parameters
  color_lock_threshold_   = get_parameter("color_lock_threshold").as_double();
  color_mismatch_penalty_ = get_parameter("color_mismatch_penalty").as_double();

  // Read Euclidean fallback parameter
  euclidean_fallback_radius_ = get_parameter("euclidean_fallback_radius").as_double();

  // Read relative-count pruning parameters
  relative_prune_enabled_          = get_parameter("relative_prune_enabled").as_bool();
  relative_prune_radius_           = get_parameter("relative_prune_radius").as_double();
  relative_prune_ratio_            = get_parameter("relative_prune_ratio").as_double();
  relative_prune_min_observations_ =
    static_cast<uint32_t>(get_parameter("relative_prune_min_observations").as_int());

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cones_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ConeMapBuilderNode::conesCallback, this, std::placeholders::_1));

  map_pub_ = create_publisher<falcon_msgs::msg::ConeArray>(
    output_map_topic_, rclcpp::QoS(1).transient_local());

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    output_marker_topic_, rclcpp::QoS(1).transient_local());

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&ConeMapBuilderNode::publishTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "ConeMapBuilder started: input='%s'  map='%s'  markers='%s'",
    input_topic_.c_str(), output_map_topic_.c_str(), output_marker_topic_.c_str());

  RCLCPP_INFO(get_logger(),
    "Color gate: lock_threshold=%.2f  mismatch_penalty=%.2f  euclidean_fallback=%.2f m",
    color_lock_threshold_, color_mismatch_penalty_, euclidean_fallback_radius_);

  if (relative_prune_enabled_) {
    RCLCPP_INFO(get_logger(),
      "Relative pruning: enabled  radius=%.2f m  ratio=%.3f  min_obs=%u",
      relative_prune_radius_, relative_prune_ratio_,
      relative_prune_min_observations_);
  } else {
    RCLCPP_INFO(get_logger(), "Relative pruning: disabled");
  }
}

// =============================================================================
// Subscription callback
// =============================================================================

void ConeMapBuilderNode::conesCallback(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  std::vector<Observation> observations;
  if (!transformCones(msg, observations)) {
    return;
  }

  const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

  std::lock_guard<std::mutex> lock(map_mutex_);
  processObservations(observations, stamp_sec);
  pruneStale(stamp_sec);
  pruneRelative();
}

// =============================================================================
// TF2 transform pipeline
// =============================================================================

bool ConeMapBuilderNode::transformCones(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr & msg,
  std::vector<Observation> & observations)
{
  const auto & source_frame = msg->header.frame_id;

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      map_frame_, source_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_s_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF lookup failed (%s -> %s): %s",
      source_frame.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }

  extractCones(msg->blue_cones,          COLOR_BLUE,       tf, observations);
  extractCones(msg->yellow_cones,        COLOR_YELLOW,     tf, observations);
  extractCones(msg->orange_cones,        COLOR_ORANGE,     tf, observations);
  extractCones(msg->big_orange_cones,    COLOR_BIG_ORANGE, tf, observations);
  extractCones(msg->unknown_color_cones, COLOR_UNKNOWN,    tf, observations);

  return true;
}

void ConeMapBuilderNode::extractCones(
  const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones,
  uint8_t color,
  const geometry_msgs::msg::TransformStamped & tf,
  std::vector<Observation> & observations)
{
  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double cy  = std::cos(yaw);
  const double sy  = std::sin(yaw);

  Eigen::Matrix2d R;
  R << cy, -sy,
       sy,  cy;

  for (const auto & cone : cones) {
    geometry_msgs::msg::PointStamped p_in, p_out;
    p_in.point = cone.point;
    tf2::doTransform(p_in, p_out, tf);

    // covariance layout: [cov_xx, cov_xy, cov_yx, cov_yy]
    Eigen::Matrix2d P_local;
    P_local << cone.covariance[0], cone.covariance[1],
               cone.covariance[2], cone.covariance[3];

    Eigen::Matrix2d P_global = R * P_local * R.transpose();
    P_global = 0.5 * (P_global + P_global.transpose());  // enforce symmetry

    Observation obs;
    obs.position   = Eigen::Vector2d(p_out.point.x, p_out.point.y);
    obs.covariance = P_global;
    obs.color      = color;
    observations.push_back(obs);
  }
}

// =============================================================================
// Data association and fusion
// =============================================================================

// -----------------------------------------------------------------------------
// processObservations
//
// Three-stage association for each incoming observation:
//
//   Stage 1 — Mahalanobis gate (primary, color-aware):
//     Finds the nearest landmark by Mahalanobis distance. Cross-color
//     observations receive a distance penalty for unsettled landmarks and
//     are hard-rejected for settled ones.
//
//   Stage 2 — Euclidean fallback (color-blind):
//     Triggered only when stage 1 rejects everything. Guards against the
//     color penalty pushing a valid spatial match above the Mahalanobis
//     threshold and causing a duplicate to be created.
//
//   Stage 3 — Create new landmark:
//     Only reached when no spatially close landmark exists at all.
// -----------------------------------------------------------------------------

void ConeMapBuilderNode::processObservations(
  const std::vector<Observation> & observations,
  double stamp_sec)
{
  for (const auto & obs : observations) {
    // Stage 1: Mahalanobis gate
    double best_distance = std::numeric_limits<double>::max();
    int idx = findNearestLandmark(obs, best_distance);

    if (idx >= 0 && best_distance < mahalanobis_threshold_) {
      fuseLandmark(landmarks_[idx], obs, stamp_sec);
      continue;
    }

    // Stage 2: Euclidean fallback — prevents duplicates caused by color penalty
    int eucl_idx = findNearestLandmarkEuclidean(obs);
    if (eucl_idx >= 0) {
      fuseLandmark(landmarks_[eucl_idx], obs, stamp_sec);
      continue;
    }

    // Stage 3: Genuinely new cone
    landmarks_.push_back(createLandmark(obs, stamp_sec));
  }
}

// -----------------------------------------------------------------------------
// findNearestLandmark
//
// Two-regime color gate:
//   Regime 1 (unsettled: confidence < color_lock_threshold_):
//     Cross-color candidates allowed with a Mahalanobis distance penalty.
//   Regime 2 (settled: confidence >= color_lock_threshold_):
//     Cross-color candidates hard-rejected.
// -----------------------------------------------------------------------------

int ConeMapBuilderNode::findNearestLandmark(
  const Observation & obs, double & best_distance) const
{
  int best_idx = -1;
  best_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const auto & lm = landmarks_[i];

    double color_penalty = 1.0;

    if (color_gated_association_ &&
        obs.color != COLOR_UNKNOWN &&
        lm.color  != COLOR_UNKNOWN &&
        obs.color != lm.color)
    {
      if (lm.confidence >= color_lock_threshold_) {
        continue;  // Regime 2: hard reject
      }
      color_penalty = color_mismatch_penalty_;  // Regime 1: soft penalty
    }

    const Eigen::Vector2d diff = obs.position - lm.position;
    const Eigen::Matrix2d S    = lm.covariance + obs.covariance;

    const double det = S.determinant();
    if (det < 1e-12) {
      continue;  // degenerate covariance
    }

    const double d2 = diff.transpose() * S.inverse() * diff;
    const double d  = std::sqrt(std::max(0.0, d2)) * color_penalty;

    if (d < best_distance) {
      best_distance = d;
      best_idx      = static_cast<int>(i);
    }
  }

  return best_idx;
}

// -----------------------------------------------------------------------------
// findNearestLandmarkEuclidean
//
// Color-blind Euclidean fallback. Returns the index of the closest landmark
// within euclidean_fallback_radius_, or -1 if none found.
// -----------------------------------------------------------------------------

int ConeMapBuilderNode::findNearestLandmarkEuclidean(const Observation & obs) const
{
  int    best_idx  = -1;
  double best_dist = euclidean_fallback_radius_;  // strict < gate

  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const double d = (obs.position - landmarks_[i].position).norm();
    if (d < best_dist) {
      best_dist = d;
      best_idx  = static_cast<int>(i);
    }
  }

  return best_idx;
}

void ConeMapBuilderNode::fuseLandmark(
  Landmark & lm, const Observation & obs, double stamp_sec)
{
  const Eigen::Matrix2d S = lm.covariance + obs.covariance;
  const Eigen::Matrix2d K = lm.covariance * S.inverse();

  const Eigen::Vector2d innovation = obs.position - lm.position;
  lm.position   += K * innovation;
  lm.covariance  = (Eigen::Matrix2d::Identity() - K) * lm.covariance;
  lm.covariance  = 0.5 * (lm.covariance + lm.covariance.transpose());

  lm.observation_count++;
  lm.last_seen_sec = stamp_sec;
  lm.color_votes[obs.color]++;
  lm.color = lm.dominantColor();

  lm.confidence = std::min(1.0,
    static_cast<double>(lm.observation_count) / confidence_saturation_);
}

Landmark ConeMapBuilderNode::createLandmark(
  const Observation & obs, double stamp_sec)
{
  Landmark lm;
  lm.id                = next_id_++;
  lm.color             = obs.color;
  lm.position          = obs.position;
  lm.covariance        = obs.covariance;
  lm.observation_count = 1;
  lm.last_seen_sec     = stamp_sec;
  lm.confidence        = 1.0 / confidence_saturation_;
  lm.color_votes.fill(0);
  lm.color_votes[obs.color] = 1;
  return lm;
}

// =============================================================================
// Pruning
// =============================================================================

void ConeMapBuilderNode::pruneStale(double now_sec)
{
  if (stale_timeout_s_ <= 0.0) {
    return;
  }

  landmarks_.erase(
    std::remove_if(landmarks_.begin(), landmarks_.end(),
      [&](const Landmark & lm) {
        return (now_sec - lm.last_seen_sec) > stale_timeout_s_;
      }),
    landmarks_.end());
}

// -----------------------------------------------------------------------------
// pruneRelative  (NEW)
//
// Removes landmarks that are dominated by a nearby neighbor in terms of
// observation count, indicating they are false/duplicate landmarks rather
// than real cones.
//
// Algorithm (single pass, no sorting required):
//
//   For each landmark A:
//     1. Skip if A.observation_count < relative_prune_min_observations_
//        (not old enough to judge — could be a newly-seen real cone)
//     2. Scan all other landmarks B where distance(A, B) < relative_prune_radius_
//     3. If any such B has observation_count > A.observation_count / ratio_,
//        mark A for removal
//
//   After the scan, erase all marked landmarks in one pass.
//
// Important implementation detail: we mark-then-erase rather than erasing
// inside the outer loop to avoid iterator invalidation and to ensure that
// mutual domination (two weak landmarks next to each other) is resolved
// consistently — both get checked against the same snapshot of the map.
//
// Logging: pruned landmarks are logged at DEBUG level with their ID and
// counts so the behavior can be verified during tuning without spamming
// the console at INFO level during normal operation.
// -----------------------------------------------------------------------------

void ConeMapBuilderNode::pruneRelative()
{
  if (!relative_prune_enabled_ || landmarks_.size() < 2) {
    return;
  }

  const double radius_sq = relative_prune_radius_ * relative_prune_radius_;

  // Collect indices to remove in this pass (avoids iterator invalidation)
  std::vector<size_t> to_remove;

  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const Landmark & candidate = landmarks_[i];

    // Guard: do not prune until the candidate has enough observations to
    // distinguish it from a freshly-created real landmark.
    if (candidate.observation_count < relative_prune_min_observations_) {
      continue;
    }

    // The count threshold this candidate must exceed to survive:
    // candidate survives iff no neighbor has more than (count / ratio) obs.
    // Rearranged: prune if neighbor_count > candidate_count / ratio
    //             i.e.  neighbor_count * ratio > candidate_count
    const double survival_threshold =
      static_cast<double>(candidate.observation_count) / relative_prune_ratio_;

    bool dominated = false;
    for (size_t j = 0; j < landmarks_.size(); ++j) {
      if (j == i) {
        continue;
      }

      const Landmark & neighbor = landmarks_[j];

      // Euclidean distance check (squared to avoid sqrt)
      const double dx = candidate.position.x() - neighbor.position.x();
      const double dy = candidate.position.y() - neighbor.position.y();
      if ((dx * dx + dy * dy) > radius_sq) {
        continue;
      }

      // Domination check: neighbor has proportionally far more observations
      if (static_cast<double>(neighbor.observation_count) > survival_threshold) {
        dominated = true;
        RCLCPP_DEBUG(get_logger(),
          "Relative prune: landmark %u (obs=%u) dominated by landmark %u "
          "(obs=%u) at dist=%.2f m — marking for removal",
          candidate.id,
          candidate.observation_count,
          neighbor.id,
          neighbor.observation_count,
          std::sqrt(dx * dx + dy * dy));
        break;  // one dominating neighbor is sufficient
      }
    }

    if (dominated) {
      to_remove.push_back(i);
    }
  }

  if (to_remove.empty()) {
    return;
  }

  // Erase in reverse index order so earlier indices stay valid
  // (std::remove_if with a set lookup is also fine, but index-based reverse
  // erase is simpler to reason about for a small vector)
  for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
    RCLCPP_INFO(get_logger(),
      "Relative prune: removed landmark id=%u (obs=%u)",
      landmarks_[*it].id, landmarks_[*it].observation_count);
    landmarks_.erase(landmarks_.begin() + static_cast<std::ptrdiff_t>(*it));
  }
}

// =============================================================================
// Publishing
// =============================================================================

void ConeMapBuilderNode::publishTimerCallback()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  const rclcpp::Time stamp = now();
  publishMap(stamp);
  publishMarkers(stamp);
}

void ConeMapBuilderNode::publishMap(const rclcpp::Time & stamp)
{
  falcon_msgs::msg::ConeArray msg;
  msg.header.stamp    = stamp;
  msg.header.frame_id = map_frame_;

  msg.cones.reserve(landmarks_.size());
  for (const auto & lm : landmarks_) {
    falcon_msgs::msg::Cone cone;
    cone.id         = lm.id;
    cone.color      = lm.color;
    cone.confidence = static_cast<float>(lm.confidence);

    cone.pose.pose.position.x    = lm.position.x();
    cone.pose.pose.position.y    = lm.position.y();
    cone.pose.pose.position.z    = 0.0;
    cone.pose.pose.orientation.w = 1.0;

    // 6×6 row-major covariance — fill the XY block only
    cone.pose.covariance[0]  = lm.covariance(0, 0);  // xx
    cone.pose.covariance[1]  = lm.covariance(0, 1);  // xy
    cone.pose.covariance[6]  = lm.covariance(1, 0);  // yx
    cone.pose.covariance[7]  = lm.covariance(1, 1);  // yy

    msg.cones.push_back(cone);
  }

  map_pub_->publish(msg);
}

void ConeMapBuilderNode::publishMarkers(const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray markers;
  std::set<uint32_t> current_ids;
  static const std::array<std::string, 3> ns_names = {"cones", "cone_ids", "covariance"};

  for (const auto & lm : landmarks_) {
    current_ids.insert(lm.id);

    // --- Cone body (cylinder) ---
    visualization_msgs::msg::Marker cone_mk;
    cone_mk.header.stamp    = stamp;
    cone_mk.header.frame_id = map_frame_;
    cone_mk.ns              = "cones";
    cone_mk.id              = static_cast<int>(lm.id);
    cone_mk.type            = visualization_msgs::msg::Marker::CYLINDER;
    cone_mk.action          = visualization_msgs::msg::Marker::ADD;

    cone_mk.pose.position.x    = lm.position.x();
    cone_mk.pose.position.y    = lm.position.y();
    cone_mk.pose.position.z    = cone_marker_height_ / 2.0;
    cone_mk.pose.orientation.w = 1.0;

    cone_mk.scale.x = cone_marker_radius_ * 2.0;
    cone_mk.scale.y = cone_marker_radius_ * 2.0;
    cone_mk.scale.z = cone_marker_height_;

    cone_mk.color   = colorForCone(lm.color);
    cone_mk.color.a = static_cast<float>(0.3 + 0.7 * lm.confidence);

    markers.markers.push_back(cone_mk);

    // --- ID + observation count label ---
    visualization_msgs::msg::Marker text_mk;
    text_mk.header.stamp    = stamp;
    text_mk.header.frame_id = map_frame_;
    text_mk.ns              = "cone_ids";
    text_mk.id              = static_cast<int>(lm.id);
    text_mk.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_mk.action          = visualization_msgs::msg::Marker::ADD;

    text_mk.pose.position.x    = lm.position.x();
    text_mk.pose.position.y    = lm.position.y();
    text_mk.pose.position.z    = cone_marker_height_ + 0.15;
    text_mk.pose.orientation.w = 1.0;

    text_mk.scale.z = 0.15;
    text_mk.color.r = 1.0f;
    text_mk.color.g = 1.0f;
    text_mk.color.b = 1.0f;
    text_mk.color.a = 1.0f;

    text_mk.text = std::to_string(lm.id) + " [" +
      std::to_string(lm.observation_count) + "]";

    markers.markers.push_back(text_mk);

    // --- 95% covariance ellipse ---
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(lm.covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }

    const Eigen::Vector2d eigenvalues  = solver.eigenvalues();
    const Eigen::Matrix2d eigenvectors = solver.eigenvectors();

    const double angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
    constexpr double kChi2Scale = 2.448;  // sqrt(chi2 95% CI, 2 DOF)
    const double rx = kChi2Scale * std::sqrt(std::max(0.0, eigenvalues(0)));
    const double ry = kChi2Scale * std::sqrt(std::max(0.0, eigenvalues(1)));

    visualization_msgs::msg::Marker ellipse_mk;
    ellipse_mk.header.stamp    = stamp;
    ellipse_mk.header.frame_id = map_frame_;
    ellipse_mk.ns              = "covariance";
    ellipse_mk.id              = static_cast<int>(lm.id);
    ellipse_mk.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    ellipse_mk.action          = visualization_msgs::msg::Marker::ADD;
    ellipse_mk.pose.orientation.w = 1.0;
    ellipse_mk.scale.x         = 0.02;

    ellipse_mk.color   = colorForCone(lm.color);
    ellipse_mk.color.a = 0.5f;

    const double ca = std::cos(angle);
    const double sa = std::sin(angle);
    for (int i = 0; i <= ellipse_segments_; ++i) {
      const double t  = 2.0 * M_PI * i / ellipse_segments_;
      const double ex = rx * std::cos(t);
      const double ey = ry * std::sin(t);

      geometry_msgs::msg::Point pt;
      pt.x = lm.position.x() + ex * ca - ey * sa;
      pt.y = lm.position.y() + ex * sa + ey * ca;
      pt.z = 0.01;
      ellipse_mk.points.push_back(pt);
    }

    markers.markers.push_back(ellipse_mk);
  }

  // DELETE markers for landmarks that disappeared since the last publish
  for (uint32_t old_id : prev_landmark_ids_) {
    if (current_ids.find(old_id) == current_ids.end()) {
      for (const auto & ns : ns_names) {
        visualization_msgs::msg::Marker del;
        del.header.stamp    = stamp;
        del.header.frame_id = map_frame_;
        del.ns              = ns;
        del.id              = static_cast<int>(old_id);
        del.action          = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(del);
      }
    }
  }

  prev_landmark_ids_ = current_ids;
  marker_pub_->publish(markers);
}

std_msgs::msg::ColorRGBA ConeMapBuilderNode::colorForCone(uint8_t color)
{
  std_msgs::msg::ColorRGBA c;
  c.a = 1.0f;
  switch (color) {
    case COLOR_BLUE:
      c.r = 0.0f; c.g = 0.0f; c.b = 1.0f; break;
    case COLOR_YELLOW:
      c.r = 1.0f; c.g = 1.0f; c.b = 0.0f; break;
    case COLOR_ORANGE:
      c.r = 1.0f; c.g = 0.5f; c.b = 0.0f; break;
    case COLOR_BIG_ORANGE:
      c.r = 1.0f; c.g = 0.3f; c.b = 0.0f; break;
    default:
      c.r = 0.7f; c.g = 0.7f; c.b = 0.7f; break;
  }
  return c;
}

}  // namespace falcon_cone_map_builder