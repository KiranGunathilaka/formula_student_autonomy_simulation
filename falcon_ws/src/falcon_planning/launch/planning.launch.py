"""
Planning Stack Launch File
--------------------------
Starts the path planner and pure pursuit controller.

Arguments:
  use_ground_truth  (default false)
    - false: Expects /falcon/fused_cones and /map/cone_map from perception
             stack and map builder running externally.
    - true:  Subscribes directly to /cones from the EUFS simulator (ground
             truth). Also launches cone_map_builder with /cones as input.
             Useful for development/testing without the perception stack.
  total_laps        (default 0)
    - Number of laps to drive before stopping (0 = unlimited).
  use_sim_time      (default true)
    - Important: true when running alongside EUFS simulation or rosbag.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    planning_share     = get_package_share_directory('falcon_planning')
    map_builder_share  = get_package_share_directory('falcon_cone_map_builder')

    planner_cfg     = os.path.join(planning_share,    'config', 'path_planner.yaml')
    pursuit_cfg     = os.path.join(planning_share,    'config', 'pure_pursuit.yaml')
    map_builder_cfg = os.path.join(map_builder_share, 'config', 'cone_map_builder.yaml')

    use_gt = LaunchConfiguration('use_ground_truth')
    cones_topic = PythonExpression([
        "'/cones' if '", use_gt, "' == 'true' else '/falcon/fused_cones'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_ground_truth', default_value='false',
                              description='Use /cones ground truth instead of '
                                          '/falcon/fused_cones from perception'),
        DeclareLaunchArgument('total_laps', default_value='0',
                              description='Number of laps to drive (0 = unlimited)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),

        # ------------------------------------------------------------ #
        # Path Planner                                                   #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[
                planner_cfg,
                {'cones_topic': cones_topic},
                {'total_laps': LaunchConfiguration('total_laps')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ------------------------------------------------------------ #
        # Pure Pursuit Controller                                        #
        # ------------------------------------------------------------ #
        Node(
            package='falcon_planning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[
                pursuit_cfg,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ------------------------------------------------------------ #
        # Cone Map Builder (only in ground-truth mode)                   #
        #   In perception mode, map builder is launched separately or    #
        #   as part of autonomy.launch.py.                               #
        # ------------------------------------------------------------ #
        Node(
            condition=IfCondition(use_gt),
            package='falcon_cone_map_builder',
            executable='falcon_cone_map_builder_node',
            name='falcon_cone_map_builder_node',
            output='screen',
            parameters=[
                map_builder_cfg,
                {'input_topic': '/cones'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
