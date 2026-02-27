cmake_minimum_required(VERSION 3.8)
project(falcon_vehicle_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(vehicle_comm_node src/vehicle_comm_node.cpp)
ament_target_dependencies(vehicle_comm_node rclcpp)

install(TARGETS vehicle_comm_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
