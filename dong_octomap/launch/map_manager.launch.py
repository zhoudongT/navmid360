"""Qt 地图管理 + 路径规划；可选启动 Go2 路径跟踪（d1_controller）。

与网页导航共用话题：/planned_path、/start_navigation、/stop_navigation、/cmd_vel。

用法::

    # 仅地图编辑与规划（不跟踪路径）
    ros2 launch dong_octomap map_manager.launch.py

    # 地图管理 + Go2 路径跟踪（需另开定位与 /cmd_vel 运动模式）
    ros2 launch dong_octomap map_manager.launch.py launch_path_controller:=true

完整网页导航请用: ros2 launch octo_planner go2_nav.launch.py
"""

from __future__ import annotations

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_d1_controller_params() -> dict:
    octo_planner_share = get_package_share_directory("octo_planner")
    config_path = os.path.join(octo_planner_share, "config", "nav_params_go2.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return dict(config.get("d1_controller", {}).get("ros__parameters", {}))


def generate_launch_description():
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="启动 dong_path_node",
    )
    launch_path_controller_arg = DeclareLaunchArgument(
        "launch_path_controller",
        default_value="false",
        description="启动 d1_controller（Go2 路径跟踪，发布 /cmd_vel）",
    )

    d1_controller_params = load_d1_controller_params()

    planner_node = Node(
        package="octo_planner",
        executable="dong_path_node",
        name="dong_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "edited_occupied_marker_topic": "/edited_occupied_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "loaded_map",
                "robot_radius": 0.20,
                "max_iterations": 250000,
                "snap_search_radius_cells": 13,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "enable_preblocked_costmap": True,
            }
        ],
    )

    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_path_controller")),
        parameters=[
            d1_controller_params,
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "map_frame": "map",
                "base_frame": "base_link",
                "base_frame_candidates": "base_link,body,base_footprint",
                "robot_type": "go2",
            },
        ],
    )

    map_package_manager_node = Node(
        package="dong_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
    )

    occupied_marker_node = Node(
        package="dong_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    map_save_gui_node = Node(
        package="dong_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
    )

    map_viewer_gui_node = Node(
        package="dong_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        parameters=[
            {
                "tf_parent_frame": "map",
                "tf_child_frame": "base_link",
            }
        ],
    )

    shutdown_when_viewer_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=map_viewer_gui_node,
            on_exit=[EmitEvent(event=Shutdown(reason="map_viewer_gui closed"))],
        )
    )

    return LaunchDescription(
        [
            launch_planner_arg,
            launch_path_controller_arg,
            planner_node,
            controller_node,
            map_package_manager_node,
            occupied_marker_node,
            map_save_gui_node,
            map_viewer_gui_node,
            shutdown_when_viewer_exits,
        ]
    )
