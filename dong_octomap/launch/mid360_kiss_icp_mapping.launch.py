"""Mid-360 + KISS-ICP 建图（无先验 PCD，对标 fastlio2 建图模式）。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("dong_octomap")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "mid360_livox_driver.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "kiss_icp_localization.launch.py")
                ),
                launch_arguments={"locate_in_prior_map": "false"}.items(),
            ),
        ]
    )
