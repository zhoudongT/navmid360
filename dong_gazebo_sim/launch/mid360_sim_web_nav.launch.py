"""Gazebo Mid-360 + FAST-LIO + 本仓库 Web/OctoMap 导航联调。

需已准备地图包目录 map_package（含 .bin 等），并在 octo_planner/config/nav_params.yaml 中配置路径，
或通过 launch 参数 map_package 覆盖。

用法::

    ros2 launch dong_gazebo_sim mid360_sim_web_nav.launch.py \\
      map_package:=/path/to/your/map_package
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    octo_share = get_package_share_directory("octo_planner")
    dong_share = get_package_share_directory("dong_octomap")

    map_pkg_arg = DeclareLaunchArgument(
        "map_package",
        default_value="",
        description="已保存的 OctoMap 地图包目录（必填）",
    )
    http_port_arg = DeclareLaunchArgument("http_port", default_value="8080")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")

    slam_sim = os.path.join(sim_share, "launch", "mid360_sim_slam.launch.py")
    web_launch = os.path.join(dong_share, "launch", "web_octomap.launch.py")

    gazebo_fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_sim),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "launch_rviz": "false",
        }.items(),
    )

    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(web_launch),
        launch_arguments={
            "map_package": LaunchConfiguration("map_package"),
            "http_port": LaunchConfiguration("http_port"),
            "launch_rosbridge": "true",
            "launch_map_gui": "false",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter(name="use_sim_time", value=True))
    ld.add_action(map_pkg_arg)
    ld.add_action(http_port_arg)
    ld.add_action(gui_arg)
    ld.add_action(gazebo_fastlio)
    ld.add_action(web)
    return ld
