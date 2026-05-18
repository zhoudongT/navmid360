"""Livox Mid-360 + FAST-LIO ROS2 SLAM helper launch (mapping).

Requires in the same workspace (see README):
  - fast_lio (e.g. https://github.com/Ericsii/FAST_LIO_ROS2 ros2 branch)
  - livox_ros_driver2 (Mid-360 driver; dependency of common FAST-LIO forks)

Includes ``fast_lio`` ``mapping.launch.py`` with RViz disabled here (this launch
adds ``dong_octomap`` RViz + ``save_bin_map_gui``). Default LiDAR config file is
``mid360.yaml`` inside ``fast_lio`` (upstream default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        fast_lio_share = get_package_share_directory("fast_lio")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ROS 2 包 fast_lio。请克隆并编译 FAST_LIO_ROS2，例如：\n"
            "  git clone -b ros2 https://github.com/Ericsii/FAST_LIO_ROS2.git"
        ) from exc

    fastlio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    if not os.path.isfile(fastlio_launch):
        raise RuntimeError(
            f"未找到 {fastlio_launch}，请确认 FAST_LIO_ROS2 是否包含 mapping.launch.py"
        )

    dong_octomap_share = get_package_share_directory("dong_octomap")
    mid360_rviz = os.path.join(dong_octomap_share, "rviz", "mid360_loc.rviz")

    octomap_cfg_dir = os.path.join(dong_octomap_share, "config")

    config_file_arg = DeclareLaunchArgument(
        "fastlio_config_file",
        default_value="mid360_go2_up.yaml",
        description="YAML file name under dong_octomap/config (Go2 顶装 Mid-360)",
    )

    fastlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "rviz": "false",
            "config_path": octomap_cfg_dir,
            "config_file": LaunchConfiguration("fastlio_config_file"),
        }.items(),
    )

    save_gui = Node(
        package="dong_octomap",
        executable="save_bin_map_gui.py",
        name="save_bin_map_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", mid360_rviz],
    )

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(fastlio_include)
    ld.add_action(save_gui)
    ld.add_action(rviz_node)
    return ld
