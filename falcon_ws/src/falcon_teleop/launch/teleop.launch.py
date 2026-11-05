"""
Keyboard teleop window (start the simulation first).

  ros2 launch falcon_teleop teleop.launch.py
  ros2 launch falcon_teleop teleop.launch.py cruise_speed:=4.0 max_steering:=0.52

With falcon_bringup's sim.launch.py running, the window finds cmd_mux and drives
through it (the planner keeps driving until you press a driving key; M hands back).
Without a mux it publishes straight to /cmd.  `sim.launch.py mode:=teleop` (or
mode:=both) opens this window for you with the vehicle's steering limit.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_topic', default_value='auto',
                              description="'auto' (via cmd_mux if running, else /cmd) or a topic"),
        DeclareLaunchArgument('cruise_speed', default_value='3.0'),
        DeclareLaunchArgument('max_speed', default_value='10.0'),
        DeclareLaunchArgument('max_steering', default_value='0.3665',
                              description='rad; ADS-DV AI limit is 21 deg, EUFS car 0.52'),
        DeclareLaunchArgument('image_topic', default_value='/teleop_camera/image_raw'),

        Node(package='falcon_teleop', executable='keyboard_teleop', name='keyboard_teleop',
             output='screen',
             parameters=[{
                 'cmd_topic': LaunchConfiguration('cmd_topic'),
                 'cruise_speed': LaunchConfiguration('cruise_speed'),
                 'max_speed': LaunchConfiguration('max_speed'),
                 'max_steering': LaunchConfiguration('max_steering'),
                 'image_topic': LaunchConfiguration('image_topic'),
             }]),
    ])
