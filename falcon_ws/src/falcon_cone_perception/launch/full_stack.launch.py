from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    eufs_launcher_pkg = FindPackageShare("eufs_launcher")
    falcon_cone_perception_pkg = FindPackageShare("falcon_cone_perception")
    foxglove_bridge_pkg = FindPackageShare("foxglove_bridge")

    eufs_launch = PathJoinSubstitution([
        eufs_launcher_pkg,
        "eufs_launcher.launch.py"
    ])

    camera_detection_launch = PathJoinSubstitution([
        falcon_cone_perception_pkg,
        "launch",
        "camera_cone_detection.launch.py"
    ])

    foxglove_launch = PathJoinSubstitution([
        foxglove_bridge_pkg,
        "launch",
        "foxglove_bridge_launch.xml"
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eufs_launch)
        ),

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(foxglove_launch)
                )
            ]
        ),

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(camera_detection_launch)
                )
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="falcon_cone_perception",
                    executable="cone_depth_localizer.py",
                    name="cone_depth_localizer",
                    output="screen",
                ),
                Node(
                    package="falcon_cone_perception",
                    executable="lidar_cone_detector.py",
                    name="lidar_cone_detector",
                    output="screen",
                ),
            ]
        ),
    ])