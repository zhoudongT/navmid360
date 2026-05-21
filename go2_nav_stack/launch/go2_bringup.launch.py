"""兼容别名 → bringup_humble.launch.py"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("go2_nav_stack")
    bringup = os.path.join(share, "launch", "bringup_humble.launch.py")
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup))]
    )
