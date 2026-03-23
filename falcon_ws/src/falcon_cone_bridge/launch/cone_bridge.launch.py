from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
    ])
