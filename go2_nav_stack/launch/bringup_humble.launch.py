"""SingleMap：Livox + FAST-LIO2 + ICP + TF + Nav2 MPPI 局部跟踪。

用法::

    ros2 launch go2_nav_stack bringup_humble.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    go2_share = get_package_share_directory("go2_nav_stack")
    dong_share = get_package_share_directory("dong_octomap")
    cfg = os.path.join(go2_share, "config")
    frames_cfg = os.path.join(cfg, "frame_config.yaml")
    mppi_launch = os.path.join(go2_share, "launch", "nav2_mppi.launch.py")

    maps_root = LaunchConfiguration("maps_root")
    prior_pcd = LaunchConfiguration("prior_pcd_path")
    launch_fastlio = LaunchConfiguration("launch_fastlio")
    launch_rviz = LaunchConfiguration("launch_rviz")
    reloc_auto_start = LaunchConfiguration("reloc_auto_start")

    fastlio_launch = os.path.join(
        dong_share, "launch", "mid360_fastlio2_localization.launch.py"
    )

    fastlio2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        condition=IfCondition(launch_fastlio),
        launch_arguments={
            "prior_pcd_path": prior_pcd,
            "locate_in_prior_map": "true",
            "icp_planar_constraint": "true",
            "launch_rviz": launch_rviz,
            # base_link 与 imu_link 重合，避免经 livox 外参导致 base 滞后/偏移
            "publish_base_tf": "true",
            "send_odom_base_tf": "false",
            "enable_transform_publisher": "false",
            "odom_output_topic": "/lio_odom_raw",
            # Go2 导航由 tf_manager 发 odom->base，需 FAST-LIO 持续更新 /lio_odom_raw
            "prior_map_freeze_odom_without_features": "false",
            # 转圈时放宽先验匹配，减少有效点骤降导致 odom 乱跳
            "prior_map_match_dist_sq": "81.0",
            "prior_map_min_effective_points": "12",
        }.items(),
    )

    stack = [
        # 里程计由 tf_manager 订阅 /lio_odom_raw 并转发 /state_estimation（见 tf_manager.yaml）
        Node(
            package="go2_nav_stack",
            executable="map_cloud_display_node",
            output="screen",
            parameters=[frames_cfg, os.path.join(cfg, "map_cloud_display.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="tf_manager_node",
            output="screen",
            parameters=[frames_cfg, os.path.join(cfg, "tf_manager.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="scan_context_manager",
            output="screen",
            parameters=[
                os.path.join(cfg, "relocalization.yaml"),
                {"maps_root": maps_root},
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="go2_nav_stack",
                    executable="relocalization_manager",
                    output="screen",
                    parameters=[
                        os.path.join(cfg, "relocalization.yaml"),
                        {
                            "auto_reloc_on_start": ParameterValue(
                                reloc_auto_start, value_type=bool
                            ),
                            "accept_manual_initialpose": True,
                        },
                    ],
                ),
            ],
        ),
        Node(
            package="go2_nav_stack",
            executable="single_map_manager",
            output="screen",
            parameters=[
                os.path.join(cfg, "single_map_manager.yaml"),
                {"maps_root": maps_root},
            ],
        ),
        Node(
            package="go2_nav_stack",
            executable="global_path_server",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mppi_launch),
        ),
        Node(
            package="go2_nav_stack",
            executable="local_path_display",
            output="screen",
            parameters=[os.path.join(cfg, "local_path_display.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="mppi_path_follower",
            output="screen",
            parameters=[os.path.join(cfg, "mppi_path_follower.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="navigation_state_machine",
            output="screen",
        ),
    ]

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("maps_root", default_value="/home/allen/maps"))
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_path",
            default_value="/home/allen/maps/scans_0.5m.pcd",
        )
    )
    ld.add_action(DeclareLaunchArgument("launch_fastlio", default_value="true"))
    ld.add_action(DeclareLaunchArgument("launch_rviz", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "reloc_auto_start",
            default_value="false",
            description="true=开机 Scan Context+ICP；false=仅 Qt「当前姿态」手动 /initialpose",
        )
    )
    ld.add_action(fastlio2)
    for n in stack:
        ld.add_action(n)
    return ld
