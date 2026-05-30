"""Go2 实机：CMU FAR Planner 替代 EGO 局部/路径规划。

前置::

  bash scripts/sync_far_planner.sh
  bash scripts/build_far_planner_humble.sh
  source install/setup.bash

FAR 输出 /way_point，由 far_waypoint_follower 转 /cmd_vel。
/planned_path 由 far_path_follower 前视发布 /goal_point，贴近 OctoMap 全局路线。
点云经 far_terrain_bridge 转为 /terrain_map 等（CMU 栈接口）。

参考:
  https://github.com/MichaelFYang/far_planner
  https://www.cmu-exploration.com/
"""

from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    try:
        get_package_share_directory("far_planner")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 far_planner。请先:\n"
            "  bash scripts/sync_far_planner.sh\n"
            "  bash scripts/build_far_planner_humble.sh\n"
            "  source install/setup.bash"
        ) from exc

    go2_share = get_package_share_directory("go2_nav_stack")
    cfg = os.path.join(go2_share, "config", "far_planner_go2.yaml")
    decoder_cfg = os.path.join(go2_share, "config", "graph_decoder_go2.yaml")

    nodes = [
        Node(
            package="graph_decoder",
            executable="graph_decoder",
            name="graph_decoder",
            output="screen",
            parameters=[decoder_cfg],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="far_planner",
            executable="far_planner",
            name="far_planner",
            output="screen",
            parameters=[cfg],
            respawn=True,
            respawn_delay=2.0,
            remappings=[
                ("/odom_world", "/state_estimation"),
                ("/terrain_cloud", "/terrain_map_ext"),
                ("/scan_cloud", "/terrain_map"),
                ("/terrain_local_cloud", "/registered_scan"),
            ],
        ),
        Node(
            package="go2_nav_stack",
            executable="far_terrain_bridge",
            output="screen",
            parameters=[cfg],
        ),
        Node(
            package="go2_nav_stack",
            executable="far_waypoint_follower",
            output="screen",
            parameters=[cfg],
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package="go2_nav_stack",
            executable="far_goal_bridge",
            output="screen",
            parameters=[cfg],
        ),
        Node(
            package="go2_nav_stack",
            executable="far_path_follower",
            output="screen",
            parameters=[cfg],
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_launch_setup),
        ]
    )
