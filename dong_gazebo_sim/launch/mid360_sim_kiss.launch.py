"""[已弃用] 请改用 ``mid360_sim_fastlio2.launch.py``（FAST-LIO2-Localization）。"""

from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    fl2 = os.path.join(sim_share, "launch", "mid360_sim_fastlio2.launch.py")
    return LaunchDescription(
        [
            LogInfo(msg="mid360_sim_kiss 已弃用，自动转发到 mid360_sim_fastlio2"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(fl2)),
        ]
    )
