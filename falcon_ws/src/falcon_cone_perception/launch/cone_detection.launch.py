from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for cone detection node."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('falcon_cone_perception')
    
    # Config files
    yolo_config = os.path.join(pkg_dir, 'config', 'yolo_detector.yaml')
    topic_remaps = os.path.join(pkg_dir, 'config', 'topic_remaps.yaml')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Cone detection node
    cone_detection_node = Node(
        package='falcon_cone_perception',
        executable='cone_detection_node',
        name='cone_detection',
        parameters=[yolo_config],
        remappings=[
            # Add topic remappings here from topic_remaps.yaml
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    return LaunchDescription([
        use_sim_time,
        cone_detection_node,
    ])
