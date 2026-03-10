"""
Full Autonomy Stack Launch
--------------------------
Starts the EUFS simulation (small_track) together with the complete
Falcon planning & control pipeline:

  EUFS sim  →  cone_bridge  →  cone_fusion  →  path_planner
                                               ↓
                                          pure_pursuit  →  /cmd  →  EUFS sim

Arguments:
  gazebo_gui  (default true)  — show Gazebo window
  rviz        (default true)  — launch RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # Package share directories                                            #
    # ------------------------------------------------------------------ #
    bringup_share = get_package_share_directory('falcon_bringup')
    planning_share = get_package_share_directory('falcon_planning')
    eufs_launcher_share = get_package_share_directory('eufs_launcher')
    fusion_share = get_package_share_directory('falcon_cone_fusion')

    eufs_master = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))

    # ------------------------------------------------------------------ #
    # Config paths                                                         #
    # ------------------------------------------------------------------ #
    fusion_cfg = os.path.join(fusion_share, 'config', 'cone_fusion.yaml')
    planner_cfg = os.path.join(planning_share, 'config', 'path_planner.yaml')
    pursuit_cfg = os.path.join(planning_share, 'config', 'pure_pursuit.yaml')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        SetEnvironmentVariable(name='EUFS_MASTER', value=eufs_master),

        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),

        # ---------------------------------------------------------------- #
        # 1. EUFS Simulator (no_perception mode → publishes /cones)        #
        # ---------------------------------------------------------------- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(eufs_launcher_share, 'simulation.launch.py')
            ),
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

        # ---------------------------------------------------------------- #
        # 2. Cone Bridge: /cones (eufs) → /perception/cones_raw (falcon)  #
        # ---------------------------------------------------------------- #
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

        # ---------------------------------------------------------------- #
        # 3. Cone Fusion: cones_raw → cones_fused (frame relabel to odom) #
        # ---------------------------------------------------------------- #
        Node(
            package='falcon_cone_fusion',
            executable='cone_fusion_node',
            name='cone_fusion_node',
            output='screen',
            parameters=[fusion_cfg],
        ),

        # ---------------------------------------------------------------- #
        # 4. Path Planner: cones_fused → /planning/path                   #
        # ---------------------------------------------------------------- #
        Node(
            package='falcon_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[planner_cfg],
        ),

        # ---------------------------------------------------------------- #
        # 5. Pure Pursuit Controller: path + odom → /cmd                  #
        # ---------------------------------------------------------------- #
        Node(
            package='falcon_planning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[pursuit_cfg],
        ),
    ])
