"""
Cone Map Builder Launch File
----------------------------
Starts the cone map builder node to accumulate a global map of cones.

Arguments:
  use_ground_truth  (default false)
    - false: Expects /falcon/fused_cones from perception stack.
    - true:  Subscribes directly to /cones from the EUFS simulator
             (ground truth). Useful for development mapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('falcon_cone_map_builder')
    config = os.path.join(pkg_share, 'config', 'cone_map_builder.yaml')

    use_gt = LaunchConfiguration('use_ground_truth')
    input_topic = PythonExpression([
        "'/cones' if '", use_gt, "' == 'true' else '/falcon/fused_cones'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_ground_truth', default_value='false',
                              description='Use /cones ground truth instead of /falcon/fused_cones'),

        Node(
            package='falcon_cone_map_builder',
            executable='falcon_cone_map_builder_node',
            name='falcon_cone_map_builder_node',
            output='screen',
            parameters=[
                config,
                {'input_topic': input_topic},
            ],
        ),
    ])
