"""
Planning stack launch file.

Starts:
  - path_planner_node   (falcon_planning)
  - pure_pursuit_node   (falcon_planning)

Expects the following to already be running:
  - /perception/cones_fused  (from cone_bridge + cone_fusion)
  - /ground_truth/odom       (from EUFS simulator)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('falcon_planning')
    planner_cfg = os.path.join(pkg, 'config', 'path_planner.yaml')
    pursuit_cfg = os.path.join(pkg, 'config', 'pure_pursuit.yaml')

    return LaunchDescription([
        Node(
            package='falcon_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[planner_cfg],
        ),
        Node(
            package='falcon_planning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[pursuit_cfg],
        ),
    ])
