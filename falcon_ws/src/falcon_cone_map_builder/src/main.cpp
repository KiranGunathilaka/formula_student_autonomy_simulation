#include <rclcpp/rclcpp.hpp>

#include "falcon_cone_map_builder/cone_map_builder_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<falcon_cone_map_builder::ConeMapBuilderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
