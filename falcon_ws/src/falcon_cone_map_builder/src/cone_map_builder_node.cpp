#include <rclcpp/rclcpp.hpp>
#include <falcon_msgs/msg/cone_array.hpp>
#include <chrono>

namespace falcon_cone_map_builder
{

class ConeMapBuilderNode : public rclcpp::Node
{
public:
  ConeMapBuilderNode()
  : Node("cone_map_builder_node")
  {
    declare_parameter<std::string>("input_topic", "/perception/cones_fused");
    declare_parameter<std::string>("output_topic", "/map/cones_map");
    declare_parameter<std::string>("output_frame", "map");
    declare_parameter<double>("publish_rate_hz", 2.0);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    double rate_hz = get_parameter("publish_rate_hz").as_double();

    fused_sub_ = create_subscription<falcon_msgs::msg::ConeArray>(
      input_topic_, 10,
      std::bind(&ConeMapBuilderNode::fusedCallback, this, std::placeholders::_1));

    map_pub_ = create_publisher<falcon_msgs::msg::ConeArray>(output_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&ConeMapBuilderNode::timerCallback, this));
  }

private:
  void fusedCallback(const falcon_msgs::msg::ConeArray::SharedPtr msg)
  {
    last_fused_ = msg;
  }

  void timerCallback()
  {
    falcon_msgs::msg::ConeArray out;
    out.header.stamp = now();
    out.header.frame_id = output_frame_;
    out.cones.clear();
    if (last_fused_) {
      out.cones = last_fused_->cones;
    }
    map_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  falcon_msgs::msg::ConeArray::SharedPtr last_fused_;
  rclcpp::Subscription<falcon_msgs::msg::ConeArray>::SharedPtr fused_sub_;
  rclcpp::Publisher<falcon_msgs::msg::ConeArray>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<falcon_cone_map_builder::ConeMapBuilderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
