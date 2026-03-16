from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    falcon_cone_perception_pkg = FindPackageShare("falcon_cone_perception")
    foxglove_bridge_pkg = FindPackageShare("foxglove_bridge")

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
        # 1) Start Foxglove bridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(foxglove_launch)
        ),

        # 2) Start YOLO camera detection
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(camera_detection_launch)
                )
            ]
        ),

        # 3) Start camera depth localization
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="falcon_cone_perception",
                    executable="cone_depth_localizer.py",
                    name="cone_depth_localizer",
                    output="screen",
                )
            ]
        ),

        # 4) Start lidar cone detector
        TimerAction(
            period=4.5,
            actions=[
                Node(
                    package="falcon_cone_perception",
                    executable="lidar_cone_detector.py",
                    name="lidar_cone_detector",
                    output="screen",
                )
            ]
        ),

        # 5) Start cone fuser after camera + lidar nodes
        TimerAction(
            period=5.5,
            actions=[
                Node(
                    package="falcon_cone_perception",
                    executable="cone_fuser.py",
                    name="cone_fuser",
                    output="screen",
                )
            ]
        ),
    ])