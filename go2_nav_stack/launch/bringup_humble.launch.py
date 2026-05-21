"""SingleMap：Livox + KISS-ICP + TF + Nav2 MPPI 局部跟踪。

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
    mppi_launch = os.path.join(go2_share, "launch", "nav2_mppi.launch.py")

    maps_root = LaunchConfiguration("maps_root")
    prior_pcd = LaunchConfiguration("prior_pcd_path")
    launch_kiss = LaunchConfiguration("launch_kiss")
    launch_rviz = LaunchConfiguration("launch_rviz")
    reloc_auto_start = LaunchConfiguration("reloc_auto_start")

    kiss_launch = os.path.join(dong_share, "launch", "mid360_kiss_icp_localization.launch.py")

    kiss_icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kiss_launch),
        condition=IfCondition(launch_kiss),
        launch_arguments={
            "prior_pcd_path": prior_pcd,
            "locate_in_prior_map": "true",
            "icp_planar_constraint": "true",
            "launch_rviz": launch_rviz,
            "publish_kiss_odom_tf": "false",
            "publish_base_tf": "false",
            "enable_map_to_odom_seed": "false",
            "enable_transform_publisher": "false",
            "enable_kiss_reset_on_icp": "false",
            "kiss_odom_topic": "/kiss_odom_raw",
        }.items(),
    )

    stack = [
        Node(
            package="go2_nav_stack",
            executable="planar_odom_filter_node",
            output="screen",
            parameters=[os.path.join(cfg, "planar_odom_filter.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="map_cloud_display_node",
            output="screen",
            parameters=[os.path.join(cfg, "map_cloud_display.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="tf_manager_node",
            output="screen",
            parameters=[os.path.join(cfg, "tf_manager.yaml")],
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
            executable="local_costmap_display",
            output="screen",
            parameters=[os.path.join(cfg, "local_costmap_display.yaml")],
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
    ld.add_action(DeclareLaunchArgument("launch_kiss", default_value="true"))
    ld.add_action(DeclareLaunchArgument("launch_rviz", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "reloc_auto_start",
            default_value="false",
            description="true=开机 Scan Context+ICP；false=仅 Qt「当前姿态」手动 /initialpose",
        )
    )
    ld.add_action(kiss_icp)
    for n in stack:
        ld.add_action(n)
    return ld
