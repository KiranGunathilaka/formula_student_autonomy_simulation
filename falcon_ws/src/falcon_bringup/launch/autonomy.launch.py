"""
Full Autonomy Stack Launch
--------------------------
The EUFS simulation with the complete Falcon pipeline on real perception:

  sim cameras + lidar -> YOLO + depth + lidar detector + fuser -> /falcon/fused_cones
      -> cone_map_builder (/map/cone_map) -> path_planner -> pure_pursuit -> cmd_mux -> /cmd

Same as `ros2 launch falcon_bringup sim.launch.py mode:=autonomy cones:=perception`;
kept so `ros2 launch falcon_bringup autonomy.launch.py total_laps:=5` still works.

Arguments:
  gazebo_gui  (default true)  — show Gazebo window
  rviz        (default true)  — launch RViz
  total_laps  (default 0)     — number of laps to drive (0 = unlimited)
  vehicle     (default ads_dv), lidar (default hesai_atx), track (default small_track)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('total_laps', default_value='0',
                              description='Number of laps (0 = unlimited)'),
        DeclareLaunchArgument('vehicle', default_value='ads_dv', description='ads_dv | eufs'),
        DeclareLaunchArgument('lidar', default_value='hesai_atx'),
        DeclareLaunchArgument('track', default_value='small_track'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('falcon_bringup'), 'launch', 'sim.launch.py')),
            launch_arguments={
                'mode': 'autonomy',
                'cones': 'perception',
                'laps': LaunchConfiguration('total_laps'),
                'vehicle': LaunchConfiguration('vehicle'),
                'lidar': LaunchConfiguration('lidar'),
                'track': LaunchConfiguration('track'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
