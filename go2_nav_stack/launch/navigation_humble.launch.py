"""导航层：map_package_manager + 网页 8888 +（可选）Qt 地图管理器 + 规划跟踪。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _check_map_package(context, *args, **kwargs):
    path = LaunchConfiguration("map_package_dir").perform(context)
    path = os.path.abspath(os.path.expanduser(path))
    meta = os.path.join(path, "meta.yaml")
    if not os.path.isdir(path):
        raise RuntimeError(f"地图包目录不存在: {path}")
    if not os.path.isfile(meta):
        raise RuntimeError(f"地图包缺少 meta.yaml: {meta}")
    return []


def generate_launch_description():
    octo_share = get_package_share_directory("octo_planner")
    nav_launch = os.path.join(octo_share, "launch", "go2_nav.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_package_dir",
                default_value="/home/allen/maps/window_filtered_all",
                description="OctoMap 地图包（含 meta.yaml、octomap_msg.npz、layers.npz）",
            ),
            DeclareLaunchArgument(
                "launch_map_gui",
                default_value="true",
                description="启动 Qt map_viewer_gui + map_save_gui（地图管理器）",
            ),
            DeclareLaunchArgument(
                "launch_web",
                default_value="true",
                description="启动静态网页 (默认 http://127.0.0.1:8888)",
            ),
            DeclareLaunchArgument(
                "launch_rosbridge",
                default_value="true",
                description="网页需 rosbridge；先安装: sudo apt install ros-humble-rosbridge-server",
            ),
            DeclareLaunchArgument("web_http_port", default_value="8888"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            OpaqueFunction(function=_check_map_package),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "map_package_dir": LaunchConfiguration("map_package_dir"),
                    "launch_map_gui": LaunchConfiguration("launch_map_gui"),
                    "launch_web": LaunchConfiguration("launch_web"),
                    "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
                    "web_http_port": LaunchConfiguration("web_http_port"),
                    "rosbridge_port": LaunchConfiguration("rosbridge_port"),
                    "launch_localization": "false",
                }.items(),
            ),
        ]
    )
