cmake_minimum_required(VERSION 3.8)
project(falcon_cone_map_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(falcon_msgs REQUIRED)

add_executable(cone_map_builder_node src/cone_map_builder_node.cpp)
ament_target_dependencies(cone_map_builder_node rclcpp std_msgs geometry_msgs falcon_msgs)

install(TARGETS cone_map_builder_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
