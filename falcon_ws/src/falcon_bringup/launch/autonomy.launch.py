"""
Full Autonomy Stack Launch
--------------------------
Starts the EUFS simulation together with the complete Falcon autonomy pipeline:

  EUFS sim (/cones)
      │
      ├── cone_bridge → cone_fusion → ─┐
      │                                 ├── path_planner → pure_pursuit → /cmd → EUFS sim
      └── cone_map_builder (/map/cone_map) ─┘

The EUFS state machine must be in Manual Drive mode before /cmd is accepted.
An enable_manual_drive node retries the /ros_can/set_mission service until it
succeeds (replaces RQt Mission Control "Drive" button).

Arguments:
  gazebo_gui  (default true)  — show Gazebo window
  rviz        (default true)  — launch RViz
  total_laps  (default 0)     — number of laps to drive (0 = unlimited)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share       = get_package_share_directory('falcon_bringup')
    planning_share      = get_package_share_directory('falcon_planning')
    eufs_launcher_share = get_package_share_directory('eufs_launcher')
    fusion_share        = get_package_share_directory('falcon_cone_fusion')
    map_builder_share   = get_package_share_directory('falcon_cone_map_builder')

    eufs_master = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))

    fusion_cfg      = os.path.join(fusion_share,      'config', 'cone_fusion.yaml')
    planner_cfg     = os.path.join(planning_share,     'config', 'path_planner.yaml')
    pursuit_cfg     = os.path.join(planning_share,     'config', 'pure_pursuit.yaml')
    map_builder_cfg = os.path.join(map_builder_share,  'config', 'cone_map_builder.yaml')

    return LaunchDescription([
        SetEnvironmentVariable(name='EUFS_MASTER', value=eufs_master),

        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('total_laps', default_value='0',
                              description='Number of laps (0 = unlimited)'),

        # ------------------------------------------------------------ #
        # 1. EUFS Simulator                                             #
        # ------------------------------------------------------------ #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(eufs_launcher_share, 'simulation.launch.py')
            ),
            launch_arguments={
                'track': 'small_track',
                'vehicleModel': 'DynamicBicycle',
                'vehicleModelConfig': 'configDry.yaml',
                'commandMode': 'velocity',
                'robot_name': 'ads-dv',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'rviz': LaunchConfiguration('rviz'),
                'publish_gt_tf': 'true',
                'pub_ground_truth': 'true',
                'launch_group': 'no_perception',
                'show_rqt_gui': 'false',
            }.items(),
        ),

        # ------------------------------------------------------------ #
        # 2. Enable Manual Drive (so /cmd is accepted by EUFS)          #
        #    Retries until /ros_can/set_mission succeeds, then exits.   #
        # ------------------------------------------------------------ #
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='falcon_bringup',
                    executable='enable_manual_drive',
                    name='enable_manual_drive',
                    output='screen',
                ),
            ],
        ),

        # ------------------------------------------------------------ #
        # 3. Cone Bridge: /cones → /perception/cones_raw               #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_cone_bridge',
            executable='cone_bridge_node',
            name='cone_bridge_node',
            output='screen',
            parameters=[{
                'input_topic': '/cones',
                'output_topic': '/perception/cones_raw',
                'output_frame': 'odom',
            }],
        ),

        # ------------------------------------------------------------ #
        # 4. Cone Fusion: cones_raw → cones_fused                      #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_cone_fusion',
            executable='cone_fusion_node',
            name='cone_fusion_node',
            output='screen',
            parameters=[fusion_cfg],
        ),

        # ------------------------------------------------------------ #
        # 5. Cone Map Builder: /cones → /map/cone_map (map frame)      #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_cone_map_builder',
            executable='falcon_cone_map_builder_node',
            name='falcon_cone_map_builder_node',
            output='screen',
            parameters=[map_builder_cfg],
        ),

        # ------------------------------------------------------------ #
        # 6. Path Planner: cones_fused + cone_map → /planning/path     #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                planner_cfg,
                {'total_laps': LaunchConfiguration('total_laps')},
            ],
        ),

        # ------------------------------------------------------------ #
        # 7. Pure Pursuit Controller: path → /cmd                      #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[pursuit_cfg],
        ),
    ])
