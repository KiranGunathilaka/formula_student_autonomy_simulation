#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vehicle_comm_node");
  node->declare_parameter<std::string>("mode", "can");
  std::string mode = node->get_parameter("mode").as_string();
  RCLCPP_INFO(node->get_logger(), "Vehicle comm mode: %s", mode.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
