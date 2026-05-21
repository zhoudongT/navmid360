"""Mid-360 + Fast-LIO2-Localization **导航定位**（先验 PCD + ICP + FAST-LIO）。

默认加载 ``/home/allen/maps/scans_0.5m.pcd``；可覆盖 ``prior_pcd_path``。
建图请用 ``mid360_fastlio2_mapping.launch.py`` 或设 ``locate_in_prior_map:=false``。

用法::

    source /opt/ros/humble/setup.bash
    source ~/dong_3d_nav/install/setup.bash

    # 导航定位（默认先验图 + 重定位）
    ros2 launch dong_octomap mid360_fastlio2_localization.launch.py

    # 指定先验图与初值
    ros2 launch dong_octomap mid360_fastlio2_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/building2_9.pcd \\
        initial_x:=0.0 initial_y:=0.0 initial_a:=0.0

    # Go2 机身 IMU
    ros2 launch dong_octomap mid360_fastlio2_localization.launch.py imu_topic:=/imu/data
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as exc:
        raise RuntimeError("未找到 livox_ros_driver2，请先编译 Livox 驱动。") from exc

    for pkg in ("fast_lio", "icp_relocalization"):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                f"未找到 ROS 2 包 {pkg}。请编译 Fast-LIO2-Localization。"
            ) from exc

    dong_octomap_share = get_package_share_directory("dong_octomap")
    livox_driver = os.path.join(dong_octomap_share, "launch", "mid360_livox_driver.launch.py")
    fl2_launch = os.path.join(dong_octomap_share, "launch", "fastlio2_localization.launch.py")
    if not os.path.isfile(livox_driver):
        raise RuntimeError(f"未找到 {livox_driver}")
    if not os.path.isfile(fl2_launch):
        raise RuntimeError(f"未找到 {fl2_launch}")

    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="/home/allen/maps/scans_0.5m.pcd",
        description="先验 PCD（导航定位必填）",
    )
    locate_arg = DeclareLaunchArgument(
        "locate_in_prior_map",
        default_value="true",
        description="true=先验图重定位（导航）；false=建图",
    )
    imu_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu",
        description="IMU 话题",
    )
    rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="是否启动 RViz",
    )
    ix_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    iy_arg = DeclareLaunchArgument("initial_y", default_value="0.0")
    iz_arg = DeclareLaunchArgument("initial_z", default_value="0.0")
    ia_arg = DeclareLaunchArgument("initial_a", default_value="0.0")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_driver),
    )
    fl2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_launch),
        launch_arguments={
            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
            "locate_in_prior_map": LaunchConfiguration("locate_in_prior_map"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_a": LaunchConfiguration("initial_a"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(prior_arg)
    ld.add_action(locate_arg)
    ld.add_action(imu_arg)
    ld.add_action(rviz_arg)
    ld.add_action(ix_arg)
    ld.add_action(iy_arg)
    ld.add_action(iz_arg)
    ld.add_action(ia_arg)
    ld.add_action(livox)
    ld.add_action(fl2)
    return ld
