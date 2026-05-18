"""Mid-360 + Fast-LIO2 **建图**（无先验图）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_launch = os.path.join(
        get_package_share_directory("dong_octomap"),
        "launch",
        "mid360_fastlio2_localization.launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "prior_pcd_path": "",
                    "locate_in_prior_map": "false",
                }.items(),
            ),
        ]
    )
