"""
EUFS Gazebo simulation with small track, ads-dv robot, velocity control,
dry track, ground truth TFs, and simulated perception cones.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # EUFS expects EUFS_MASTER for GAZEBO_PLUGIN_PATH; derive from this package's share path
    _share = get_package_share_directory('falcon_bringup')
    _eufs_master = os.path.dirname(os.path.dirname(os.path.dirname(_share)))

    eufs_launcher_share = get_package_share_directory('eufs_launcher')
    sim_launch_path = os.path.join(eufs_launcher_share, 'simulation.launch.py')

    return LaunchDescription([
        SetEnvironmentVariable(name='EUFS_MASTER', value=_eufs_master),
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
        DeclareLaunchArgument(
            'launch_group',
            default_value='no_perception',
            description="'no_perception' (default): /cones, lidar/camera off. 'default': real lidar (/gazebo_scan) and cameras.",
        ),
        DeclareLaunchArgument(
            'show_rqt_gui',
            default_value='true',
            description='EUFS RQt mission/steering GUIs (requires Qt in the active Python env)',
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
                'launch_group': LaunchConfiguration('launch_group'),
                'show_rqt_gui': LaunchConfiguration('show_rqt_gui'),
            }.items(),
        ),
    ])
