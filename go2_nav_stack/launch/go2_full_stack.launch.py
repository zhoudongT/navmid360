"""一键：定位 bringup（EGO 局部）+ 导航（网页 + 地图管理器 + 全局规划）。"""

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
            DeclareLaunchArgument("launch_web", default_value="true"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument("web_http_port", default_value="8888"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument(
                "rosbridge_delay_sec",
                default_value="4.0",
                description="rosbridge 延迟启动（秒）；oneclick 默认 2",
            ),
            DeclareLaunchArgument(
                "reloc_auto_start",
                default_value="true",
                description="true=开机 Scan Context+ICP 快速自定位；false=手动或 seed",
            ),
            DeclareLaunchArgument(
                "reloc_startup_delay_sec",
                default_value="6.0",
                description="Livox+ICP 就绪后触发 Scan Context（秒）",
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument(
                "icp_converged_count_thre",
                default_value="8",
                description="ICP 连续收敛帧数，越小越快",
            ),
            DeclareLaunchArgument(
                "seed_initialpose",
                default_value="false",
                description="已知粗位姿时仅 ICP：延时发 /initialpose",
            ),
            DeclareLaunchArgument(
                "seed_initialpose_delay_sec",
                default_value="7.0",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup),
                launch_arguments={
                    "maps_root": LaunchConfiguration("maps_root"),
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
                    "reloc_startup_delay_sec": LaunchConfiguration(
                        "reloc_startup_delay_sec"
                    ),
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                    "icp_converged_count_thre": LaunchConfiguration(
                        "icp_converged_count_thre"
                    ),
                    "seed_initialpose": LaunchConfiguration("seed_initialpose"),
                    "seed_initialpose_delay_sec": LaunchConfiguration(
                        "seed_initialpose_delay_sec"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation),
                launch_arguments={
                    "map_package_dir": LaunchConfiguration("map_package_dir"),
                    "launch_map_gui": LaunchConfiguration("launch_map_gui"),
                    "launch_web": LaunchConfiguration("launch_web"),
                    "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
                    "web_http_port": LaunchConfiguration("web_http_port"),
                    "rosbridge_port": LaunchConfiguration("rosbridge_port"),
                    "rosbridge_delay_sec": LaunchConfiguration("rosbridge_delay_sec"),
                }.items(),
            ),
        ]
    )
