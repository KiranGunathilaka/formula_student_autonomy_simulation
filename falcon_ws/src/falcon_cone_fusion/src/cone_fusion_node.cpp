#include <rclcpp/rclcpp.hpp>
#include <falcon_msgs/msg/cone_array.hpp>

namespace falcon_cone_fusion
{

class ConeFusionNode : public rclcpp::Node
{
public:
  ConeFusionNode()
  : Node("cone_fusion_node")
  {
    declare_parameter<std::string>("input_topic", "/perception/cones_raw");
    declare_parameter<std::string>("output_topic", "/perception/cones_fused");
    declare_parameter<std::string>("output_frame", "odom");

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();

    input_sub_ = create_subscription<falcon_msgs::msg::ConeArray>(
      input_topic_, 10,
      std::bind(&ConeFusionNode::inputCallback, this, std::placeholders::_1));

    output_pub_ = create_publisher<falcon_msgs::msg::ConeArray>(output_topic_, 10);
  }

private:
  void inputCallback(const falcon_msgs::msg::ConeArray::SharedPtr msg)
  {
    falcon_msgs::msg::ConeArray out;
    out.header = msg->header;
    out.header.frame_id = output_frame_;
    out.cones = msg->cones;
    output_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  rclcpp::Subscription<falcon_msgs::msg::ConeArray>::SharedPtr input_sub_;
  rclcpp::Publisher<falcon_msgs::msg::ConeArray>::SharedPtr output_pub_;
};

}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<falcon_cone_fusion::ConeFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
