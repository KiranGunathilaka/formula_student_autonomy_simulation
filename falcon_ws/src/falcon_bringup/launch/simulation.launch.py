"""
EUFS Gazebo simulation with small track, ads-dv robot, velocity control,
dry track, ground truth TFs, and simulated perception cones.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    eufs_launcher_share = get_package_share_directory('eufs_launcher')
    sim_launch_path = os.path.join(eufs_launcher_share, 'simulation.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Launch Gazebo GUI',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
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
            }.items(),
        ),
    ])
