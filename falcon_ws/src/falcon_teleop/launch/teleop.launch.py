from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('falcon_teleop')
    install_prefix = os.path.dirname(os.path.dirname(pkg_share))
    script = os.path.join(install_prefix, 'bin', 'keyboard_teleop')
    if not os.path.isfile(script):
        script = os.path.join(install_prefix, 'lib', 'falcon_teleop', 'keyboard_teleop')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[script],
            output='screen',
            shell=False,
        ),
    ])
