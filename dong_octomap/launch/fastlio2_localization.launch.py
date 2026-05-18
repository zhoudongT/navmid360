"""Fast-LIO2-Localization（Polaris）+ Livox Mid-360，替代上游 ``example.launch.py``。

- **建图 / 看点云**（默认）：``locate_in_prior_map:=false``，直接发布 ``/cloud_registered`` 等。
- **先验图重定位**：传入 ``prior_pcd_path:=/绝对路径/map.pcd``，并保证该文件存在；
  ICP 收敛后向 ``/icp_result`` 发布初值，FAST-LIO 才开始输出。

用法::

    # 仅建图、看实时点云（无需先验 PCD）
    ros2 launch dong_octomap fastlio2_localization.launch.py

    # 先验图重定位
    ros2 launch dong_octomap fastlio2_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans.pcd locate_in_prior_map:=true

    # 使用 Go2 机身 IMU（需 unitree_ros2 发布 /imu/data）
    ros2 launch dong_octomap fastlio2_localization.launch.py imu_topic:=/imu/data
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    fast_lio_params = [
        fl_cfg,
        {
            "locate_in_prior_map": use_prior,
            "prior_map_path": prior if prior else "",
            "common.imu_topic": imu_topic,
            "filter_size_map": filter_size_map,
            "prior_map_skip_load_downsample": prior_map_skip_voxel,
            "prior_map_match_dist_sq": prior_map_match_dist_sq,
        },
    ]

    icp_share = get_package_share_directory("icp_relocalization")
    rviz_cfg = os.path.join(icp_share, "rviz", "loam_livox.rviz")
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    publish_base_tf = LaunchConfiguration("publish_base_tf").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    nodes = [
        Node(
            package="icp_relocalization",
            executable="transform_publisher",
            name="transform_publisher",
            output="screen",
        ),
    ]

    if publish_base_tf:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
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
        icp_params = {
            "map_path": prior,
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
        }
        delayed = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="icp_relocalization",
                    executable="icp_node",
                    name="icp_node",
                    output="screen",
                    parameters=[icp_params],
                ),
                Node(
                    package="fast_lio",
                    executable="fastlio_mapping",
                    parameters=fast_lio_params,
                    output="screen",
                    remappings=[("/Odometry", "/state_estimation")],
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
                remappings=[("/Odometry", "/state_estimation")],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prior_pcd_path",
                default_value="",
                description="先验 PCD（重定位模式必填，须与 fast_lio prior_map_path 一致）",
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
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument(
                "fitness_score_thre",
                default_value="0.25",
                description="ICP fitness 阈值（越小越严，典型 0.15~0.3）",
            ),
            DeclareLaunchArgument(
                "converged_count_thre",
                default_value="10",
                description="连续满足阈值的帧数后才发布 /icp_result",
            ),
            DeclareLaunchArgument(
                "max_correspondence_distance",
                default_value="0.5",
                description="ICP 最大对应距离 (m)",
            ),
            DeclareLaunchArgument(
                "map_voxel_leaf_size",
                default_value="0.5",
                description="ICP 先验图体素 (m)，越大点越少、加载越快",
            ),
            DeclareLaunchArgument("cloud_voxel_leaf_size", default_value="0.3"),
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
                default_value="15.0",
                description="先验模式近邻距离平方阈值（15≈3.9m，25≈5m）",
            ),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="true",
                description="发布 base_link->imu_link 静态 TF（无 go2 URDF 时需要）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
