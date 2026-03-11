#include "falcon_cone_map_builder/cone_map_builder_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace falcon_cone_map_builder
{

ConeMapBuilderNode::ConeMapBuilderNode(const rclcpp::NodeOptions & options)
: Node("falcon_cone_map_builder_node", options), next_id_(0)
{
  declare_parameter<std::string>("input_topic", "/cones");
  declare_parameter<std::string>("output_map_topic", "/map/cone_map");
  declare_parameter<std::string>("output_marker_topic", "/map/cone_markers");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<double>("mahalanobis_threshold", 3.0);
  declare_parameter<double>("publish_rate_hz", 10.0);
  declare_parameter<double>("stale_timeout_s", 0.0);
  declare_parameter<int>("confidence_saturation", 15);
  declare_parameter<double>("cone_marker_height", 0.3);
  declare_parameter<double>("cone_marker_radius", 0.1);
  declare_parameter<bool>("color_gated_association", true);
  declare_parameter<double>("tf_timeout_s", 0.1);
  declare_parameter<int>("ellipse_segments", 32);

  input_topic_           = get_parameter("input_topic").as_string();
  output_map_topic_      = get_parameter("output_map_topic").as_string();
  output_marker_topic_   = get_parameter("output_marker_topic").as_string();
  map_frame_             = get_parameter("map_frame").as_string();
  mahalanobis_threshold_ = get_parameter("mahalanobis_threshold").as_double();
  publish_rate_hz_       = get_parameter("publish_rate_hz").as_double();
  stale_timeout_s_       = get_parameter("stale_timeout_s").as_double();
  confidence_saturation_ =
    static_cast<uint32_t>(get_parameter("confidence_saturation").as_int());
  cone_marker_height_       = get_parameter("cone_marker_height").as_double();
  cone_marker_radius_       = get_parameter("cone_marker_radius").as_double();
  color_gated_association_  = get_parameter("color_gated_association").as_bool();
  tf_timeout_s_             = get_parameter("tf_timeout_s").as_double();
  ellipse_segments_         = get_parameter("ellipse_segments").as_int();

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
    "ConeMapBuilder started: subscribing to '%s', publishing map on '%s', markers on '%s'",
    input_topic_.c_str(), output_map_topic_.c_str(), output_marker_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Subscription callback
// ---------------------------------------------------------------------------

void ConeMapBuilderNode::conesCallback(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  std::vector<Observation> observations;
  if (!transformCones(msg, observations)) {
    return;
  }

  double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

  std::lock_guard<std::mutex> lock(map_mutex_);
  processObservations(observations, stamp_sec);
  pruneStale(stamp_sec);
}

// ---------------------------------------------------------------------------
// TF2 transform pipeline
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Data association and fusion
// ---------------------------------------------------------------------------

void ConeMapBuilderNode::processObservations(
  const std::vector<Observation> & observations,
  double stamp_sec)
{
  for (const auto & obs : observations) {
    double best_distance = std::numeric_limits<double>::max();
    int idx = findNearestLandmark(obs, best_distance);

    if (idx >= 0 && best_distance < mahalanobis_threshold_) {
      fuseLandmark(landmarks_[idx], obs, stamp_sec);
    } else {
      landmarks_.push_back(createLandmark(obs, stamp_sec));
    }
  }
}

int ConeMapBuilderNode::findNearestLandmark(
  const Observation & obs, double & best_distance) const
{
  int best_idx = -1;
  best_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const auto & lm = landmarks_[i];

    // Color gate: only associate same-color cones (unknown matches anything)
    if (color_gated_association_ &&
        obs.color != COLOR_UNKNOWN && lm.color != COLOR_UNKNOWN &&
        obs.color != lm.color) {
      continue;
    }

    Eigen::Vector2d diff = obs.position - lm.position;
    Eigen::Matrix2d S = lm.covariance + obs.covariance;

    double det = S.determinant();
    if (det < 1e-12) {
      continue;
    }

    double d2 = diff.transpose() * S.inverse() * diff;
    double d  = std::sqrt(std::max(0.0, d2));

    if (d < best_distance) {
      best_distance = d;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

void ConeMapBuilderNode::fuseLandmark(
  Landmark & lm, const Observation & obs, double stamp_sec)
{
  // Kalman-style covariance-weighted fusion
  Eigen::Matrix2d S = lm.covariance + obs.covariance;
  Eigen::Matrix2d K = lm.covariance * S.inverse();

  Eigen::Vector2d innovation = obs.position - lm.position;
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

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void ConeMapBuilderNode::publishTimerCallback()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  rclcpp::Time stamp = now();
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

    // 6x6 row-major covariance: fill the XY block
    // indices: [row * 6 + col]
    cone.pose.covariance[0]  = lm.covariance(0, 0);  // (0,0) xx
    cone.pose.covariance[1]  = lm.covariance(0, 1);  // (0,1) xy
    cone.pose.covariance[6]  = lm.covariance(1, 0);  // (1,0) yx
    cone.pose.covariance[7]  = lm.covariance(1, 1);  // (1,1) yy

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
    constexpr double kChi2Scale = 2.448;
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

  // Send DELETE for landmarks that were in the previous publish but are now gone
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
      c.r = 0.0f; c.g = 0.0f; c.b = 1.0f;
      break;
    case COLOR_YELLOW:
      c.r = 1.0f; c.g = 1.0f; c.b = 0.0f;
      break;
    case COLOR_ORANGE:
      c.r = 1.0f; c.g = 0.5f; c.b = 0.0f;
      break;
    case COLOR_BIG_ORANGE:
      c.r = 1.0f; c.g = 0.3f; c.b = 0.0f;
      break;
    default:
      c.r = 0.7f; c.g = 0.7f; c.b = 0.7f;
      break;
  }
  return c;
}

}  // namespace falcon_cone_map_builder
