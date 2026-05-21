"""一键：定位 bringup（Nav2 MPPI 局部）+ 导航（网页 + 地图管理器 + 全局规划）。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("go2_nav_stack")
    bringup = os.path.join(share, "launch", "bringup_humble.launch.py")
    navigation = os.path.join(share, "launch", "navigation_humble.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("maps_root", default_value="/home/allen/maps"),
            DeclareLaunchArgument(
                "prior_pcd_path",
                default_value="/home/allen/maps/scans_0.5m.pcd",
            ),
            DeclareLaunchArgument(
                "map_package_dir",
                default_value="/home/allen/maps/window_filtered_all",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_map_gui", default_value="true"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument("web_http_port", default_value="8080"),
            DeclareLaunchArgument(
                "reloc_auto_start",
                default_value="false",
                description="true=开机自动重定位；false=Qt「当前姿态」手动点击",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup),
                launch_arguments={
                    "maps_root": LaunchConfiguration("maps_root"),
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation),
                launch_arguments={
                    "map_package_dir": LaunchConfiguration("map_package_dir"),
                    "launch_map_gui": LaunchConfiguration("launch_map_gui"),
                    "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
                    "web_http_port": LaunchConfiguration("web_http_port"),
                }.items(),
            ),
        ]
    )
