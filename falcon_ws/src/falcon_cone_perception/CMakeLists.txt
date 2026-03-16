cmake_minimum_required(VERSION 3.8)
project(falcon_cone_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cone_depth_localizer.py
  scripts/lidar_cone_detector.py
  scripts/cone_fuser.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()