"""
Full Autonomy Stack Launch
--------------------------
Starts the EUFS simulation together with the complete Falcon autonomy pipeline:

  EUFS sim (cameras + LiDAR)
      │
      ├─ perception stack ─→ /falcon/fused_cones (eufs_msgs) ─┐
      │                                                        ├→ path_planner → pure_pursuit → /cmd
      └── cone_map_builder (/map/cone_map) ───────────────────┘

The perception stack (YOLO + depth + LiDAR + cone fuser) publishes fused
cones on /falcon/fused_cones in eufs_msgs format.  The cone_map_builder
accumulates a global map on /map/cone_map.  The path_planner merges live
cones with the map and publishes /planning/path.

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
                'launch_group': 'default',
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
        #    Output: /falcon/fused_cones (eufs_msgs)                    #
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
        #    Subscribes to /falcon/fused_cones and builds a global      #
        #    landmark map published on /map/cone_map.                   #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_cone_map_builder',
            executable='falcon_cone_map_builder_node',
            name='falcon_cone_map_builder_node',
            output='screen',
            parameters=[
                map_builder_cfg,
                {'use_sim_time': True},
            ],
        ),

        # ------------------------------------------------------------ #
        # 5. Path Planner                                               #
        #    Subscribes to /falcon/fused_cones (live) and               #
        #    /map/cone_map (accumulated map). Publishes /planning/path. #
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
