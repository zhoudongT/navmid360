"""KISS-ICP + ICP 先验图重定位（Mid-360），流程对齐 ``fastlio2_localization.launch.py``。

- **建图**：``locate_in_prior_map:=false``，KISS-ICP 发布 ``odom`` TF 与 ``/state_estimation``。
- **先验图重定位**：ICP 发布 ``map→odom``；KISS 发布动态 ``odom→base_link``（机身随行走更新）。
  勿再静态 ``odom→base``，否则路径对但 ``d1_controller`` 认为机身朝向不变会猛打角速度。

需另起 Livox（``mid360_livox_driver.launch.py``）或 ``mid360_kiss_icp_*.launch.py``。

用法::

    ros2 launch dong_octomap kiss_icp_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd locate_in_prior_map:=true
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


def _kiss_icp_node(
    kiss_cfg: str,
    use_prior: bool,
    publish_base_tf: bool,
    launch_rviz: bool,
    pointcloud_topic: str,
    kiss_odom_topic: str,
    publish_odom_tf: bool = True,
    enable_transform_publisher: bool = True,
    enable_kiss_reset_on_icp: bool = True,
):
    dong_share = get_package_share_directory("dong_octomap")
    icp_share = get_package_share_directory("icp_relocalization")
    kiss_share = get_package_share_directory("kiss_icp")

    # go2_nav_stack tf_manager 可接管 odom→base_link（publish_kiss_odom_tf:=false）
    kiss_base_frame = "base_link"
    # 导航栈 Scan Context / 局部图需要 /cloud_registered
    kiss_debug_clouds = True
    kiss_params = [
        kiss_cfg,
        {
            "base_frame": kiss_base_frame,
            "lidar_odom_frame": "odom",
            "publish_odom_tf": publish_odom_tf,
            "invert_odom_tf": False,
            "publish_debug_clouds": kiss_debug_clouds,
            "use_sim_time": False,
        },
    ]

    nodes = []
    if enable_transform_publisher:
        nodes.append(
            Node(
                package="icp_relocalization",
                executable="transform_publisher",
                name="transform_publisher",
                output="screen",
            )
        )

    if publish_base_tf:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="livox_to_imu_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "livox_frame", "imu_link"],
            )
        )
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            )
        )

    nodes.extend(
        [
            Node(
                package="dong_octomap",
                executable="livox_custommsg_to_pointcloud2.py",
                name="livox_custommsg_to_pointcloud2",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "output_topic": pointcloud_topic,
                        "output_frame_id": "livox_frame",
                    }
                ],
            ),
            Node(
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                parameters=kiss_params,
                remappings=[
                    ("pointcloud_topic", pointcloud_topic),
                    ("kiss/odometry", kiss_odom_topic),
                    ("kiss/frame", "/cloud_registered"),
                    ("kiss/local_map", "/Laser_map"),
                ],
            ),
        ]
    )
    if enable_kiss_reset_on_icp:
        nodes.append(
            Node(
                package="dong_octomap",
                executable="kiss_icp_reset_on_icp.py",
                name="kiss_icp_reset_on_icp",
                output="screen",
            )
        )

    if launch_rviz:
        rviz_cfg = os.path.join(icp_share, "rviz", "loam_livox.rviz")
        if os.path.isfile(rviz_cfg):
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_cfg],
                    output="screen",
                )
            )
        else:
            kiss_rviz = os.path.join(kiss_share, "rviz", "kiss_icp.rviz")
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", kiss_rviz],
                    output="screen",
                )
            )

    return nodes


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

    publish_base_tf = LaunchConfiguration("publish_base_tf").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    publish_odom_tf = LaunchConfiguration("publish_kiss_odom_tf").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    pointcloud_topic = LaunchConfiguration("pointcloud_topic").perform(context).strip()
    kiss_odom_topic = LaunchConfiguration("kiss_odom_topic").perform(context).strip()
    enable_map_seed = LaunchConfiguration("enable_map_to_odom_seed").perform(
        context
    ).lower() in ("1", "true", "yes", "on")
    enable_tf_pub = LaunchConfiguration("enable_transform_publisher").perform(
        context
    ).lower() in ("1", "true", "yes", "on")
    enable_kiss_reset = LaunchConfiguration("enable_kiss_reset_on_icp").perform(
        context
    ).lower() in ("1", "true", "yes", "on")

    dong_share = get_package_share_directory("dong_octomap")
    kiss_cfg = os.path.join(dong_share, "config", "kiss_icp_mid360.yaml")
    icp_yaml = os.path.join(dong_share, "config", "icp_fastlio_localization.yaml")

    nodes = []

    if use_prior:
        ix = float(LaunchConfiguration("initial_x").perform(context))
        iy = float(LaunchConfiguration("initial_y").perform(context))
        iz = float(LaunchConfiguration("initial_z").perform(context))
        ia = float(LaunchConfiguration("initial_a").perform(context))
        # 可选：静态 map→odom 种子（与 ICP 动态 TF 冲突，Go2 导航默认关闭）
        if enable_map_seed:
            nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="map_to_odom_seed_tf",
                    arguments=[
                        str(ix),
                        str(iy),
                        str(iz),
                        str(ia),
                        "0",
                        "0",
                        "map",
                        "odom",
                    ],
                )
            )
        icp_params = {
            "map_path": prior,
            "initial_x": float(LaunchConfiguration("initial_x").perform(context)),
            "initial_y": float(LaunchConfiguration("initial_y").perform(context)),
            "initial_z": float(LaunchConfiguration("initial_z").perform(context)),
            "initial_a": float(LaunchConfiguration("initial_a").perform(context)),
            "map_voxel_leaf_size": float(
                LaunchConfiguration("map_voxel_leaf_size").perform(context)
            ),
            "cloud_voxel_leaf_size": float(
                LaunchConfiguration("cloud_voxel_leaf_size").perform(context)
            ),
            "map_frame_id": "map",
            "pcl_type": "livox",
            "fitness_score_thre": float(
                LaunchConfiguration("fitness_score_thre").perform(context)
            ),
            "converged_count_thre": int(
                LaunchConfiguration("converged_count_thre").perform(context)
            ),
            "max_correspondence_distance": float(
                LaunchConfiguration("max_correspondence_distance").perform(context)
            ),
            "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint")
            .perform(context)
            .lower()
            in ("1", "true", "yes", "on"),
            # Go2 栈由 tf_manager 发布 map->odom，ICP 只发 /icp_result
            "publish_map_to_odom_tf": False,
        }
        delayed = TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="icp_relocalization",
                    executable="icp_node",
                    name="icp_node",
                    output="screen",
                    parameters=[icp_yaml, icp_params],
                ),
            ],
        )
        nodes.append(delayed)

    nodes.extend(
        _kiss_icp_node(
            kiss_cfg,
            use_prior,
            publish_base_tf,
            launch_rviz,
            pointcloud_topic,
            kiss_odom_topic,
            publish_odom_tf,
            enable_tf_pub,
            enable_kiss_reset,
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
                description="先验 PCD（重定位）",
            ),
            DeclareLaunchArgument(
                "locate_in_prior_map",
                default_value="false",
                description="true：ICP 初值 + 静态 map→odom；false：纯 KISS-ICP 建图",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/livox/lidar_pcd2",
                description="KISS-ICP 输入 PointCloud2（由 livox_custommsg_to_pointcloud2 发布）",
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
            DeclareLaunchArgument("fitness_score_thre", default_value="0.08"),
            DeclareLaunchArgument("converged_count_thre", default_value="12"),
            DeclareLaunchArgument("max_correspondence_distance", default_value="0.3"),
            DeclareLaunchArgument("map_voxel_leaf_size", default_value="0.5"),
            DeclareLaunchArgument("cloud_voxel_leaf_size", default_value="0.2"),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="true",
                description="静态 base→imu；先验模式另发 imu→livox，KISS 发 odom→livox_frame",
            ),
            DeclareLaunchArgument(
                "publish_kiss_odom_tf",
                default_value="true",
                description="KISS 发布 odom→base_link；go2 bringup 建议 false 由 tf_manager 发布",
            ),
            DeclareLaunchArgument(
                "icp_planar_constraint",
                default_value="true",
                description="ICP 仅平面位姿，减轻朝天雷达误估 pitch",
            ),
            DeclareLaunchArgument(
                "enable_map_to_odom_seed",
                default_value="false",
                description="静态 map→odom 种子；与 ICP 连续跟踪冲突，Go2 请保持 false",
            ),
            DeclareLaunchArgument(
                "enable_transform_publisher",
                default_value="false",
                description="icp_result 静态 map→odom；Go2 由 icp_node 动态发布",
            ),
            DeclareLaunchArgument(
                "enable_kiss_reset_on_icp",
                default_value="false",
                description="ICP 收敛后 reset KISS；会打断 odom 连续性，Go2 请保持 false",
            ),
            DeclareLaunchArgument(
                "kiss_odom_topic",
                default_value="/kiss_odom_raw",
                description="KISS 原始里程计话题；Go2 经 planar_odom_filter 输出 /state_estimation",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
