"""
Full Autonomy Stack with Real Perception
-----------------------------------------
Starts the EUFS simulation with real sensor data (cameras + LiDAR) together
with the complete Falcon perception and planning pipeline.

  EUFS sim (cameras, LiDAR)
      │
      ├─ perception stack ─┐
      │   (YOLO + depth    │
      │    + LiDAR + fuser)│
      │                    │
      │   /falcon/fused_cones (eufs_msgs) ──→ cone_map_builder ──→ /map/cone_map ─┐
      │                    │                                                       │
      │   /perception/cones_fused (falcon_msgs) ──→ path_planner ←────────────────┘
      │                                                  │
      └──────────────────── pure_pursuit ← /planning/path
                                │
                            /cmd → EUFS sim

The perception cone_fuser publishes both eufs_msgs (for the map builder) and
falcon_msgs (for the path planner) formats directly, eliminating the need for
falcon_cone_bridge and falcon_cone_fusion nodes.

Arguments:
  gazebo_gui   (default true)      — show Gazebo window
  rviz         (default true)      — launch RViz
  total_laps   (default 0)         — number of laps to drive (0 = unlimited)
  launch_group (default 'default') — EUFS sensor group ('default' enables
                                     cameras + LiDAR; 'no_perception' gives
                                     pseudo /cones only)
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
    perception_share    = get_package_share_directory('falcon_cone_perception')
    eufs_launcher_share = get_package_share_directory('eufs_launcher')
    map_builder_share   = get_package_share_directory('falcon_cone_map_builder')

    eufs_master = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))

    planner_cfg     = os.path.join(planning_share,    'config', 'path_planner.yaml')
    pursuit_cfg     = os.path.join(planning_share,    'config', 'pure_pursuit.yaml')
    map_builder_cfg = os.path.join(map_builder_share, 'config', 'cone_map_builder.yaml')

    perception_launch = os.path.join(
        perception_share, 'launch', 'perception.launch.py')

    return LaunchDescription([
        SetEnvironmentVariable(name='EUFS_MASTER', value=eufs_master),

        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('total_laps', default_value='0',
                              description='Number of laps (0 = unlimited)'),
        DeclareLaunchArgument('launch_group', default_value='default',
                              description="'default': cameras + LiDAR enabled. "
                                          "'no_perception': pseudo /cones only."),

        # ------------------------------------------------------------ #
        # 1. EUFS Simulator (default launch group = real sensors)       #
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
                'robot_name': 'eufs',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'rviz': LaunchConfiguration('rviz'),
                'publish_gt_tf': 'true',
                'pub_ground_truth': 'true',
                'launch_group': LaunchConfiguration('launch_group'),
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
        # 3. Perception Stack                                           #
        #    YOLO camera detection, depth localizer, LiDAR detector,    #
        #    cone fuser — with internal staged delays.                  #
        #    Outputs:                                                   #
        #      /falcon/fused_cones       (eufs_msgs)                    #
        #      /perception/cones_fused   (falcon_msgs)                  #
        # ------------------------------------------------------------ #
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(perception_launch),
                ),
            ],
        ),

        # ------------------------------------------------------------ #
        # 4. Cone Map Builder                                           #
        #    Subscribes to /falcon/fused_cones (perception output) and  #
        #    builds a global landmark map published on /map/cone_map.   #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_cone_map_builder',
            executable='falcon_cone_map_builder_node',
            name='falcon_cone_map_builder_node',
            output='screen',
            parameters=[
                map_builder_cfg,
                {'input_topic': '/falcon/fused_cones'},
                {'use_sim_time': True},
            ],
        ),

        # ------------------------------------------------------------ #
        # 5. Path Planner                                               #
        #    Merges live /perception/cones_fused with /map/cone_map     #
        #    and publishes /planning/path.                              #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                planner_cfg,
                {'total_laps': LaunchConfiguration('total_laps')},
                {'use_sim_time': True},
            ],
        ),

        # ------------------------------------------------------------ #
        # 6. Pure Pursuit Controller: path → /cmd                       #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[
                pursuit_cfg,
                {'use_sim_time': True},
            ],
        ),
    ])
