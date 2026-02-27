#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <falcon_msgs/msg/cone_array.hpp>
#include <chrono>

namespace falcon_cone_perception
{

class ConePerceptionNode : public rclcpp::Node
{
public:
  ConePerceptionNode()
  : Node("cone_perception_node")
  {
    declare_parameter<std::string>("pointcloud_topic", "/points");
    declare_parameter<std::string>("output_topic", "/perception/cones_raw");
    declare_parameter<std::string>("output_frame", "base_link");
    declare_parameter<double>("publish_rate_hz", 10.0);

    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    double rate_hz = get_parameter("publish_rate_hz").as_double();

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&ConePerceptionNode::pointcloudCallback, this, std::placeholders::_1));

    cones_pub_ = create_publisher<falcon_msgs::msg::ConeArray>(output_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&ConePerceptionNode::timerCallback, this));
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    (void)msg;
  }

  void timerCallback()
  {
    falcon_msgs::msg::ConeArray array;
    array.header.stamp = now();
    array.header.frame_id = output_frame_;
    array.cones.clear();
    cones_pub_->publish(array);
  }

  std::string pointcloud_topic_;
  std::string output_topic_;
  std::string output_frame_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<falcon_msgs::msg::ConeArray>::SharedPtr cones_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<falcon_cone_perception::ConePerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
