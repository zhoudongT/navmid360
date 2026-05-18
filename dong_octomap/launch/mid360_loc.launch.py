"""Livox Mid-360 + FAST-LIO ROS2 localization / monitoring launch.

Includes FAST-LIO mapping node (same as SLAM fork default) and optional PCD
overlay for RViz. Choose a ``.pcd`` when prompted (e.g. prior map for display).

Requires ``fast_lio`` in the workspace (see ``mid360_slam.launch.py`` / README).
"""

import os
import tkinter as tk
from tkinter import filedialog

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def choose_relocalization_pcd_file() -> str:
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    default_dir = os.path.expanduser("~/maps")
    if not os.path.isdir(default_dir):
        default_dir = os.path.expanduser("~")
    selected_file = filedialog.askopenfilename(
        title="选择要在 RViz 中叠加显示的 PCD 点云（可选参考图）",
        initialdir=default_dir,
        filetypes=[("PCD Files", "*.pcd"), ("All Files", "*.*")],
    )
    root.destroy()
    if not selected_file:
        raise RuntimeError("未选择 .pcd 文件，已取消启动。")
    return os.path.abspath(selected_file)


def generate_launch_description():
    try:
        fast_lio_share = get_package_share_directory("fast_lio")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ROS 2 包 fast_lio。请克隆并编译 FAST_LIO_ROS2（ros2 分支）。"
        ) from exc

    fastlio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    if not os.path.isfile(fastlio_launch):
        raise RuntimeError(f"未找到 {fastlio_launch}")

    dong_octomap_share = get_package_share_directory("dong_octomap")
    mid360_rviz = os.path.join(dong_octomap_share, "rviz", "mid360_loc.rviz")

    relocalization_pcd_file = choose_relocalization_pcd_file()

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

    pcd_publisher_node = Node(
        package="dong_octomap",
        executable="pcd_file_publisher.py",
        name="pcd_file_publisher",
        output="screen",
        parameters=[
            {
                "pcd_path": relocalization_pcd_file,
                "topic": "/pcd_points",
                "frame_id": "camera_init",
                "publish_hz": 1.0,
            }
        ],
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
    ld.add_action(pcd_publisher_node)
    ld.add_action(rviz_node)
    return ld
