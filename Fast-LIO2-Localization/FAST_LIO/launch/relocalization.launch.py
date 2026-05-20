"""先验图重定位（ICP + FAST-LIO），不含 Livox 驱动。

需已发布 ``/livox/lidar``（另开 ``mid360_livox_driver`` 或使用 ``dong_octomap`` 一键 launch）。

推荐一键（Mid-360 + 定位）::

    ros2 launch dong_octomap mid360_fastlio2_localization.launch.py

仅 FAST-LIO + ICP（雷达已在外部启动）::

    ros2 launch fast_lio relocalization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sanitize_ld_library_path() -> str:
    """海康 MVS 自带的 libusb 过旧，会导致 libpcl_io 报 libusb_set_option 未定义。"""
    parts = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p and "/opt/MVS" not in p]
    if "/lib/x86_64-linux-gnu" not in parts:
        parts.insert(0, "/lib/x86_64-linux-gnu")
    return ":".join(parts)


def _setup(context, *args, **kwargs):
    prior = LaunchConfiguration("prior_pcd_path").perform(context).strip()
    if not prior:
        raise RuntimeError("必须设置 prior_pcd_path:=/绝对路径/xxx.pcd")
    if not os.path.isfile(prior):
        raise RuntimeError(f"先验地图不存在: {prior}")

    fl_cfg = os.path.join(
        get_package_share_directory("fast_lio"),
        "config",
        "fast_lio_relocalization_param.yaml",
    )
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    use_skip_voxel = "_0.5m" in prior or "_0.3m" in prior
    fast_lio_params = [
        fl_cfg,
        {
            "locate_in_prior_map": True,
            "prior_map_path": prior,
            "common.imu_topic": imu_topic,
            "prior_map_skip_load_downsample": use_skip_voxel,
            "prior_map_match_dist_sq": 15.0,
            "filter_size_map": 0.5,
        },
    ]

    icp_params = {
        "map_path": prior,
        "initial_x": float(LaunchConfiguration("initial_x").perform(context)),
        "initial_y": float(LaunchConfiguration("initial_y").perform(context)),
        "initial_z": float(LaunchConfiguration("initial_z").perform(context)),
        "initial_a": float(LaunchConfiguration("initial_a").perform(context)),
        "map_voxel_leaf_size": 0.5,
        "cloud_voxel_leaf_size": 0.3,
        "map_frame_id": "map",
        "pcl_type": "livox",
        "fitness_score_thre": 0.25,
        "converged_count_thre": 10,
        "max_correspondence_distance": 0.5,
    }

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        ),
        Node(
            package="icp_relocalization",
            executable="transform_publisher",
            name="transform_publisher",
            output="screen",
        ),
        Node(
            package="icp_relocalization",
            executable="icp_node",
            name="icp_node",
            output="screen",
            parameters=[icp_params],
        ),
    ]

    if launch_rviz:
        rviz_cfg = os.path.join(
            get_package_share_directory("icp_relocalization"),
            "rviz",
            "loam_livox.rviz",
        )
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                output="screen",
            )
        )

    nodes.append(
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="fast_lio",
                    executable="fastlio_mapping",
                    parameters=fast_lio_params,
                    output="screen",
                    remappings=[("/Odometry", "/state_estimation")],
                ),
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=_sanitize_ld_library_path(),
            ),
            DeclareLaunchArgument(
                "prior_pcd_path",
                default_value="/home/allen/maps/scans_0.5m.pcd",
                description="先验 PCD；勿用 67M 点的 scans.pcd（会卡死），推荐 scans_0.5m.pcd",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/livox/imu",
                description="IMU 话题；Go2 机身 IMU 可用 /imu/data",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="是否启动 RViz",
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
