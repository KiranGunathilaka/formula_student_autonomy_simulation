from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_cone_perception = get_package_share_directory('falcon_cone_perception')
    pkg_cone_fusion = get_package_share_directory('falcon_cone_fusion')
    pkg_cone_map_builder = get_package_share_directory('falcon_cone_map_builder')
    pkg_sim = get_package_share_directory('falcon_sim')

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    sim_enable_fused_arg = DeclareLaunchArgument('sim_enable_fused', default_value='true')
    sim_enable_map_arg = DeclareLaunchArgument('sim_enable_map', default_value='true')

    cone_perception_config = os.path.join(pkg_cone_perception, 'config', 'cone_perception.yaml')
    cone_fusion_config = os.path.join(pkg_cone_fusion, 'config', 'cone_fusion.yaml')
    cone_map_builder_config = os.path.join(pkg_cone_map_builder, 'config', 'cone_map_builder.yaml')
    sim_config = os.path.join(pkg_sim, 'config', 'sim.yaml')

    cone_perception_node = Node(
        package='falcon_cone_perception',
        executable='cone_perception_node',
        name='cone_perception_node',
        parameters=[cone_perception_config],
        condition=UnlessCondition(LaunchConfiguration('use_sim', default='false')),
    )

    cone_fusion_node = Node(
        package='falcon_cone_fusion',
        executable='cone_fusion_node',
        name='cone_fusion_node',
        parameters=[cone_fusion_config],
    )

    cone_map_builder_node = Node(
        package='falcon_cone_map_builder',
        executable='cone_map_builder_node',
        name='cone_map_builder_node',
        parameters=[cone_map_builder_config],
    )

    sim_node = Node(
        package='falcon_sim',
        executable='falcon_sim_node',
        name='falcon_sim_node',
        parameters=[
            sim_config,
            {'enable_fused': LaunchConfiguration('sim_enable_fused', default='true')},
            {'enable_map': LaunchConfiguration('sim_enable_map', default='true')},
        ],
        condition=IfCondition(LaunchConfiguration('use_sim', default='false')),
    )

    nodes = [
        use_sim_arg,
        sim_enable_fused_arg,
        sim_enable_map_arg,
    ]

    ld = LaunchDescription(nodes)

    ld.add_action(cone_perception_node)
    ld.add_action(cone_fusion_node)
    ld.add_action(cone_map_builder_node)
    ld.add_action(sim_node)

    return ld
