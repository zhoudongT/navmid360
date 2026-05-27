"""Fast-LIO2-Localization（Polaris）+ Livox Mid-360，替代上游 ``example.launch.py``。

- **建图 / 看点云**（默认）：``locate_in_prior_map:=false``，直接发布 ``/cloud_registered`` 等。
- **先验图重定位**：传入 ``prior_pcd_path:=/绝对路径/map.pcd``，并保证该文件存在；
  ICP 收敛后向 ``/icp_result`` 发布初值，FAST-LIO 才开始输出。

用法::

    # 仅建图、看实时点云（无需先验 PCD）
    ros2 launch dong_octomap fastlio2_localization.launch.py

    # 先验图重定位（ICP 默认见 config/icp_fastlio_localization.yaml）
    ros2 launch dong_octomap fastlio2_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd locate_in_prior_map:=true

    # 更严 ICP（fitness 约 0.05 时）
    ros2 launch dong_octomap fastlio2_localization.launch.py locate_in_prior_map:=true \\
        fitness_score_thre:=0.06 max_correspondence_distance:=0.25 cloud_voxel_leaf_size:=0.15

    # 使用 Go2 机身 IMU（需 unitree_ros2 发布 /imu/data）
    ros2 launch dong_octomap fastlio2_localization.launch.py imu_topic:=/imu/data
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ld_library_path_for_pcl_icp() -> str:
    system_lib = "/lib/x86_64-linux-gnu"
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in existing.split(":") if p]
    if parts and parts[0] == system_lib:
        return existing
    if system_lib in parts:
        parts = [system_lib] + [p for p in parts if p != system_lib]
    else:
        parts = [system_lib] + parts
    return ":".join(parts)


def _launch_setup(context, *args, **kwargs):
    prior = LaunchConfiguration("prior_pcd_path").perform(context).strip()
    locate = LaunchConfiguration("locate_in_prior_map").perform(context).strip().lower()
    use_prior = locate in ("1", "true", "yes", "on")
    if use_prior and not prior:
        raise RuntimeError(
            "locate_in_prior_map=true 时必须设置 prior_pcd_path:=/绝对路径/xxx.pcd"
        )
    if prior and not os.path.isfile(prior):
        raise RuntimeError(f"先验地图不存在: {prior}")

    fast_lio_share = get_package_share_directory("fast_lio")
    fl_cfg = os.path.join(fast_lio_share, "config", "fast_lio_relocalization_param.yaml")
    imu_topic = LaunchConfiguration("imu_topic").perform(context)

    filter_size_map = float(LaunchConfiguration("filter_size_map").perform(context))
    prior_map_skip_voxel = LaunchConfiguration("prior_map_skip_load_downsample").perform(
        context
    ).lower() in ("1", "true", "yes", "on")
    prior_map_match_dist_sq = float(
        LaunchConfiguration("prior_map_match_dist_sq").perform(context)
    )
    prior_map_min_eff = int(LaunchConfiguration("prior_map_min_effective_points").perform(context))
    freeze_odom_cfg = LaunchConfiguration(
        "prior_map_freeze_odom_without_features"
    ).perform(context).strip().lower()
    prior_map_freeze_odom = freeze_odom_cfg in ("1", "true", "yes", "on")
    publish_base_tf = LaunchConfiguration("publish_base_tf").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    send_odom_tf_cfg = LaunchConfiguration("send_odom_base_tf").perform(context).strip().lower()
    if send_odom_tf_cfg in ("1", "true", "yes", "on"):
        send_odom_base_tf = True
    elif send_odom_tf_cfg in ("0", "false", "no", "off"):
        send_odom_base_tf = False
    else:
        send_odom_base_tf = not (use_prior and publish_base_tf)

    enable_tf_pub = LaunchConfiguration("enable_transform_publisher").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    odom_out = LaunchConfiguration("odom_output_topic").perform(context).strip() or "/state_estimation"

    fast_lio_params = [
        fl_cfg,
        {
            "locate_in_prior_map": use_prior,
            "prior_map_path": prior if prior else "",
            "common.imu_topic": imu_topic,
            "filter_size_map": filter_size_map,
            "prior_map_skip_load_downsample": prior_map_skip_voxel,
            "prior_map_match_dist_sq": prior_map_match_dist_sq,
            "prior_map_freeze_odom_without_features": prior_map_freeze_odom,
            "prior_map_min_effective_points": prior_map_min_eff,
            "common.send_odom_base_tf": send_odom_base_tf,
        },
    ]
    overlay = LaunchConfiguration("fastlio_config_overlay").perform(context).strip()
    if overlay:
        if not os.path.isfile(overlay):
            raise RuntimeError(f"FAST-LIO overlay 配置不存在: {overlay}")
        fast_lio_params.append(overlay)

    icp_share = get_package_share_directory("icp_relocalization")
    rviz_cfg = LaunchConfiguration("rviz_config").perform(context).strip()
    if not rviz_cfg:
        rviz_cfg = os.path.join(icp_share, "rviz", "loam_livox.rviz")
    elif not os.path.isfile(rviz_cfg):
        raise RuntimeError(f"RViz 配置文件不存在: {rviz_cfg}")
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    nodes = []
    if enable_tf_pub:
        nodes.append(
            Node(
                package="icp_relocalization",
                executable="transform_publisher",
                name="transform_publisher",
                output="screen",
            )
        )

    # go2_nav_stack：tf_manager 发 base->livox；此处仅 livox->imu（FAST-LIO sensor_frame）
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="livox_to_imu_static_tf",
            arguments=["0", "0", "0", "0", "0", "0", "livox_frame", "imu_link"],
        )
    )
    if publish_base_tf:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            )
        )
        if use_prior:
            nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="odom_to_base_static_tf",
                    arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
                )
            )

    if launch_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                output="screen",
            )
        )

    if use_prior:
        dong_share = get_package_share_directory("dong_octomap")
        icp_yaml = os.path.join(dong_share, "config", "icp_fastlio_localization.yaml")
        icp_params = {
            "map_path": prior,
            "imu_topic": imu_topic,
            "initial_x": float(LaunchConfiguration("initial_x").perform(context)),
            "initial_y": float(LaunchConfiguration("initial_y").perform(context)),
            "initial_z": float(LaunchConfiguration("initial_z").perform(context)),
            "initial_a": float(LaunchConfiguration("initial_a").perform(context)),
            "map_voxel_leaf_size": float(LaunchConfiguration("map_voxel_leaf_size").perform(context)),
            "cloud_voxel_leaf_size": float(LaunchConfiguration("cloud_voxel_leaf_size").perform(context)),
            "map_frame_id": "map",
            "pcl_type": "livox",
            "fitness_score_thre": float(LaunchConfiguration("fitness_score_thre").perform(context)),
            "converged_count_thre": int(LaunchConfiguration("converged_count_thre").perform(context)),
            "max_correspondence_distance": float(
                LaunchConfiguration("max_correspondence_distance").perform(context)
            ),
            "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint").perform(context).lower()
            in ("1", "true", "yes", "on"),
        }
        icp_param_files: list = [icp_yaml, icp_params]
        icp_overlay = LaunchConfiguration("icp_overlay_config").perform(context).strip()
        if icp_overlay:
            if not os.path.isfile(icp_overlay):
                raise RuntimeError(f"ICP overlay 配置不存在: {icp_overlay}")
            icp_param_files.append(icp_overlay)
        delayed = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="icp_relocalization",
                    executable="icp_node",
                    name="icp_node",
                    output="screen",
                    parameters=icp_param_files,
                ),
                Node(
                    package="fast_lio",
                    executable="fastlio_mapping",
                    parameters=fast_lio_params,
                    output="screen",
                    remappings=[("/Odometry", odom_out)],
                ),
            ],
        )
        nodes.append(delayed)
    else:
        nodes.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                parameters=fast_lio_params,
                output="screen",
                remappings=[("/Odometry", odom_out)],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=_ld_library_path_for_pcl_icp(),
            ),
            DeclareLaunchArgument(
                "prior_pcd_path",
                default_value="/home/allen/maps/scans_0.5m.pcd",
                description="先验 PCD（重定位；默认 scans_0.5m.pcd，须与 fast_lio prior_map_path 一致）",
            ),
            DeclareLaunchArgument(
                "locate_in_prior_map",
                default_value="false",
                description="true：等 ICP 初值后再跑 FAST-LIO；false：直接建图并发布点云",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/livox/imu",
                description="IMU 话题；Go2 机身 IMU 可设为 /imu/data",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="是否启动 RViz（若 Ogre 崩溃可设 false，另开 rviz2）",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="RViz2 配置 .rviz 绝对路径；空则用 icp loam_livox.rviz",
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument(
                "fitness_score_thre",
                default_value="0.08",
                description="ICP fitness 上限（越小越严；你当前约 0.05 可设 0.06~0.10）",
            ),
            DeclareLaunchArgument(
                "converged_count_thre",
                default_value="12",
                description="连续满足阈值的帧数后才发布 /icp_result",
            ),
            DeclareLaunchArgument(
                "max_correspondence_distance",
                default_value="0.3",
                description="ICP 最大对应距离 (m)，越小匹配越紧",
            ),
            DeclareLaunchArgument(
                "map_voxel_leaf_size",
                default_value="0.5",
                description="ICP 先验图体素 (m)，与 scans_0.5m.pcd 一致",
            ),
            DeclareLaunchArgument(
                "cloud_voxel_leaf_size",
                default_value="0.2",
                description="ICP 实时扫描体素 (m)，越小点越密",
            ),
            DeclareLaunchArgument(
                "filter_size_map",
                default_value="0.5",
                description="FAST-LIO 先验图体素 (m)，已离线降采样时配合 prior_map_skip_load_downsample",
            ),
            DeclareLaunchArgument(
                "prior_map_skip_load_downsample",
                default_value="true",
                description="先验 PCD 已体素降采样时不再二次滤波",
            ),
            DeclareLaunchArgument(
                "prior_map_match_dist_sq",
                default_value="49.0",
                description="先验模式近邻距离平方阈值（49≈7m；无有效点时配合 freeze_odom）",
            ),
            DeclareLaunchArgument(
                "prior_map_min_effective_points",
                default_value="15",
                description="先验模式至少多少有效点才更新 odom TF（否则冻结防漂）",
            ),
            DeclareLaunchArgument(
                "prior_map_freeze_odom_without_features",
                default_value="true",
                description="ICP 锁定后冻结 FAST-LIO 里程计；go2_nav_stack 请 false",
            ),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="true",
                description="发布 livox_frame->imu_link、base_link->imu_link 静态 TF",
            ),
            DeclareLaunchArgument(
                "send_odom_base_tf",
                default_value="",
                description="FAST-LIO 是否发布 odom->base；go2_nav_stack 请设 false，由 tf_manager 发布",
            ),
            DeclareLaunchArgument(
                "enable_transform_publisher",
                default_value="true",
                description="是否启动 icp transform_publisher；go2_nav_stack 请设 false",
            ),
            DeclareLaunchArgument(
                "odom_output_topic",
                default_value="/state_estimation",
                description="FAST-LIO /Odometry 重映射话题；go2 导航栈常用 /lio_odom_raw",
            ),
            DeclareLaunchArgument(
                "icp_planar_constraint",
                default_value="true",
                description="ICP 仅输出平面位姿，避免朝天雷达误估 pitch 导致点云翘起",
            ),
            DeclareLaunchArgument(
                "fastlio_config_overlay",
                default_value="",
                description="FAST-LIO 额外 YAML（Go2 先验图运动鲁棒参数）",
            ),
            DeclareLaunchArgument(
                "icp_overlay_config",
                default_value="",
                description="icp_node 额外 YAML（如 Go2 continuous_tracking）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
