"""Mid-360 + KISS-ICP + ICP 先验图重定位（对标 ``mid360_fastlio2_localization.launch.py``）。

用法::

    source /opt/ros/humble/setup.bash
    source install/setup.bash

    ros2 launch dong_octomap mid360_kiss_icp_localization.launch.py

    ros2 launch dong_octomap mid360_kiss_icp_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd initial_x:=0 initial_y:=0
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
    for pkg in ("livox_ros_driver2", "kiss_icp", "icp_relocalization"):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                f"未找到 {pkg}。请运行: bash scripts/sync_kiss_icp.sh && "
                "bash scripts/build_kiss_icp_humble.sh，并编译 icp_relocalization / livox。"
            ) from exc

    dong_share = get_package_share_directory("dong_octomap")
    livox_driver = os.path.join(dong_share, "launch", "mid360_livox_driver.launch.py")
    kiss_launch = os.path.join(dong_share, "launch", "kiss_icp_localization.launch.py")

    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="/home/allen/maps/scans_0.5m.pcd",
    )
    locate_arg = DeclareLaunchArgument("locate_in_prior_map", default_value="true")
    icp_planar_arg = DeclareLaunchArgument("icp_planar_constraint", default_value="true")
    rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")
    ix_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    iy_arg = DeclareLaunchArgument("initial_y", default_value="0.0")
    iz_arg = DeclareLaunchArgument("initial_z", default_value="0.0")
    ia_arg = DeclareLaunchArgument("initial_a", default_value="0.0")
    kiss_odom_tf_arg = DeclareLaunchArgument("publish_kiss_odom_tf", default_value="true")

    livox = IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_driver))
    kiss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kiss_launch),
        launch_arguments={
            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
            "locate_in_prior_map": LaunchConfiguration("locate_in_prior_map"),
            "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_a": LaunchConfiguration("initial_a"),
            "publish_kiss_odom_tf": LaunchConfiguration("publish_kiss_odom_tf"),
        }.items(),
    )

    ld = LaunchDescription()
    for a in (
        prior_arg,
        locate_arg,
        icp_planar_arg,
        rviz_arg,
        ix_arg,
        iy_arg,
        iz_arg,
        ia_arg,
        kiss_odom_tf_arg,
    ):
        ld.add_action(a)
    ld.add_action(livox)
    ld.add_action(kiss)
    return ld
