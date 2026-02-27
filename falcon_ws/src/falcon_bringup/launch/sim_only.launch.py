from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory('falcon_sim')
    sim_config = os.path.join(pkg_sim, 'config', 'sim.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('enable_fused', default_value='true'),
        DeclareLaunchArgument('enable_map', default_value='true'),
        DeclareLaunchArgument('fused_topic', default_value='/perception/cones_fused'),
        DeclareLaunchArgument('map_topic', default_value='/map/cones_map'),
        DeclareLaunchArgument('fused_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('map_rate_hz', default_value='1.0'),
    ])

    sim_node = Node(
        package='falcon_sim',
        executable='falcon_sim_node',
        name='falcon_sim_node',
        parameters=[
            sim_config,
            {'enable_fused': LaunchConfiguration('enable_fused', default='true')},
            {'enable_map': LaunchConfiguration('enable_map', default='true')},
            {'fused_topic': LaunchConfiguration('fused_topic', default='/perception/cones_fused')},
            {'map_topic': LaunchConfiguration('map_topic', default='/map/cones_map')},
            {'fused_rate_hz': LaunchConfiguration('fused_rate_hz', default='10.0')},
            {'map_rate_hz': LaunchConfiguration('map_rate_hz', default='1.0')},
        ],
    )
    ld.add_action(sim_node)
    return ld
