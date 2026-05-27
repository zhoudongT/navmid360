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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    imu_filter_arg = DeclareLaunchArgument(
        "imu_filter_enable",
        default_value="true",
        description="是否对 /livox/imu 做卡尔曼滤波并发布 /livox/imu_filtered",
    )
    imu_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu_filtered",
        description="FAST-LIO / ICP 使用的 IMU 话题（滤波开启时默认 /livox/imu_filtered）",
    )
    rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="是否启动 RViz",
    )
    rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="RViz2 配置路径；go2_nav_stack 一键启动会传入 rviz_go2_nav.rviz",
    )
    ix_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    iy_arg = DeclareLaunchArgument("initial_y", default_value="0.0")
    iz_arg = DeclareLaunchArgument("initial_z", default_value="0.0")
    ia_arg = DeclareLaunchArgument("initial_a", default_value="0.0")
    icp_planar_arg = DeclareLaunchArgument("icp_planar_constraint", default_value="true")
    pub_base_arg = DeclareLaunchArgument("publish_base_tf", default_value="true")
    send_odom_arg = DeclareLaunchArgument("send_odom_base_tf", default_value="")
    tf_pub_arg = DeclareLaunchArgument("enable_transform_publisher", default_value="true")
    odom_out_arg = DeclareLaunchArgument("odom_output_topic", default_value="/state_estimation")
    freeze_odom_arg = DeclareLaunchArgument(
        "prior_map_freeze_odom_without_features",
        default_value="true",
        description="先验图 ICP 锁定后是否冻结 FAST-LIO 里程计；go2_nav_stack 请 false",
    )
    match_dist_arg = DeclareLaunchArgument(
        "prior_map_match_dist_sq",
        default_value="49.0",
        description="先验图匹配距离平方阈值（81≈9m，转圈时可放宽）",
    )
    min_eff_arg = DeclareLaunchArgument(
        "prior_map_min_effective_points",
        default_value="15",
        description="至少多少有效点才更新 LIO 状态",
    )
    conv_arg = DeclareLaunchArgument(
        "converged_count_thre",
        default_value="12",
        description="ICP 连续收敛帧数阈值",
    )
    fl_overlay_arg = DeclareLaunchArgument(
        "fastlio_config_overlay",
        default_value="",
        description="FAST-LIO 额外 YAML overlay",
    )
    icp_overlay_arg = DeclareLaunchArgument(
        "icp_overlay_config",
        default_value="",
        description="icp_node 额外 YAML overlay",
    )

    try:
        go2_share = get_package_share_directory("go2_nav_stack")
        imu_filter_cfg = os.path.join(go2_share, "config", "livox_imu_filter.yaml")
    except PackageNotFoundError:
        imu_filter_cfg = ""

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_driver),
    )
    imu_filter_node = Node(
        package="go2_nav_stack",
        executable="livox_imu_kalman_filter",
        name="livox_imu_kalman_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("imu_filter_enable")),
        parameters=[imu_filter_cfg] if imu_filter_cfg else [],
    )
    fl2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_launch),
        launch_arguments={
            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
            "locate_in_prior_map": LaunchConfiguration("locate_in_prior_map"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_a": LaunchConfiguration("initial_a"),
            "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
            "publish_base_tf": LaunchConfiguration("publish_base_tf"),
            "send_odom_base_tf": LaunchConfiguration("send_odom_base_tf"),
            "enable_transform_publisher": LaunchConfiguration("enable_transform_publisher"),
            "odom_output_topic": LaunchConfiguration("odom_output_topic"),
            "prior_map_freeze_odom_without_features": LaunchConfiguration(
                "prior_map_freeze_odom_without_features"
            ),
            "prior_map_match_dist_sq": LaunchConfiguration("prior_map_match_dist_sq"),
            "prior_map_min_effective_points": LaunchConfiguration(
                "prior_map_min_effective_points"
            ),
            "converged_count_thre": LaunchConfiguration("converged_count_thre"),
            "fastlio_config_overlay": LaunchConfiguration("fastlio_config_overlay"),
            "icp_overlay_config": LaunchConfiguration("icp_overlay_config"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(prior_arg)
    ld.add_action(locate_arg)
    ld.add_action(imu_filter_arg)
    ld.add_action(imu_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_cfg_arg)
    ld.add_action(ix_arg)
    ld.add_action(iy_arg)
    ld.add_action(iz_arg)
    ld.add_action(ia_arg)
    ld.add_action(icp_planar_arg)
    ld.add_action(pub_base_arg)
    ld.add_action(send_odom_arg)
    ld.add_action(tf_pub_arg)
    ld.add_action(odom_out_arg)
    ld.add_action(freeze_odom_arg)
    ld.add_action(match_dist_arg)
    ld.add_action(min_eff_arg)
    ld.add_action(conv_arg)
    ld.add_action(fl_overlay_arg)
    ld.add_action(icp_overlay_arg)
    ld.add_action(livox)
    ld.add_action(imu_filter_node)
    ld.add_action(fl2)
    return ld
