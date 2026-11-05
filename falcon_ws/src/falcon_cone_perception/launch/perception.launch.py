"""
Perception stack: YOLO on the camera, depth localiser, lidar cone detector, fuser.

Arguments:
  lidar_detector  3d (default): PointCloud2 on /lidar/points (falcon_description cars)
                  2d: LaserScan on /gazebo_scan (the old EUFS RPLidar URDFs)
  body_box        "x_min,x_max,y_half,z_max" of the car in base_footprint; lidar
                  points inside it are dropped (sim.launch.py passes the vehicle's)
  foxglove        start foxglove_bridge (default false)
Output: /falcon/fused_cones (eufs_msgs/ConeArrayWithCovariance, base_footprint)
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _lidar_detector(context):
    kind = LaunchConfiguration('lidar_detector').perform(context)
    if kind == '2d':
        return [Node(package='falcon_cone_perception', executable='lidar_cone_detector.py',
                     name='lidar_cone_detector', output='screen',
                     parameters=[{'use_sim_time': True}])]
    box = [float(v) for v in LaunchConfiguration('body_box').perform(context).split(',')]
    return [Node(package='falcon_cone_perception', executable='lidar_cone_detector_3d.py',
                 name='lidar_cone_detector', output='screen',
                 parameters=[{'use_sim_time': True, 'body_box': box}])]


def generate_launch_description():
    pkg = FindPackageShare('falcon_cone_perception')
    camera_detection_launch = PathJoinSubstitution([pkg, 'launch', 'camera_cone_detection.launch.py'])
    foxglove_launch = PathJoinSubstitution(
        [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_detector', default_value='3d'),
        DeclareLaunchArgument('body_box', default_value='-1.35,1.50,0.72,0.85'),
        DeclareLaunchArgument('foxglove', default_value='false'),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(foxglove_launch),
                                 condition=IfCondition(LaunchConfiguration('foxglove'))),

        # YOLO camera detection
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_detection_launch))]),

        # Camera depth localisation
        TimerAction(period=4.0, actions=[
            Node(package='falcon_cone_perception', executable='cone_depth_localizer.py',
                 name='cone_depth_localizer', output='screen',
                 parameters=[{'use_sim_time': True}])]),

        # Lidar cone detector
        TimerAction(period=4.5, actions=[OpaqueFunction(function=_lidar_detector)]),

        # Cone fuser after camera + lidar nodes
        TimerAction(period=5.5, actions=[
            Node(package='falcon_cone_perception', executable='cone_fuser.py',
                 name='cone_fuser', output='screen',
                 parameters=[{'use_sim_time': True}])]),
    ])
