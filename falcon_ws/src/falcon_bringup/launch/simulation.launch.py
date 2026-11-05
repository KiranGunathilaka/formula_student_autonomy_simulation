"""
EUFS Gazebo simulation with the Falcon car and nothing driving it.

Kept for the older two-terminal workflow (this, then planning.launch.py or
teleop.launch.py); it now runs sim.launch.py mode:=none.

  launch_group:=no_perception (default): simulated cones on /cones, lidar and camera off
  launch_group:=default: lidar (/lidar/points) and camera (/zed/...) on as well

Use `ros2 launch falcon_bringup sim.launch.py` for everything in one command.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    sensors = PythonExpression(
        ["'false' if '", LaunchConfiguration('launch_group'), "' == 'no_perception' else 'true'"])
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='Launch Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('launch_group', default_value='no_perception',
                              description="'no_perception': /cones only; 'default': lidar and camera too"),
        DeclareLaunchArgument('vehicle', default_value='ads_dv', description='ads_dv | eufs'),
        DeclareLaunchArgument('lidar', default_value='hesai_atx'),
        DeclareLaunchArgument('track', default_value='small_track'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('falcon_bringup'), 'launch', 'sim.launch.py')),
            launch_arguments={
                'mode': 'none',
                'vehicle': LaunchConfiguration('vehicle'),
                'lidar': LaunchConfiguration('lidar'),
                'track': LaunchConfiguration('track'),
                'use_lidar': sensors,
                'use_camera': sensors,
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
