"""Go2 完整导航栈：OctoMap 规划 + 路径跟踪 + 网页 +（可选）Qt 地图管理器。

定位需单独启动（map/odom/base_link）::

    ros2 launch dong_octomap mid360_fastlio2_localization.launch.py \\
        prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd locate_in_prior_map:=true

导航（网页 http://localhost:8080 + 路径跟踪）::

    ros2 launch octo_planner go2_nav.launch.py \\
        map_package_dir:=/path/to/octomap_package

带 Qt 地图管理器（与网页相同话题，可二选一或同时用）::

    ros2 launch octo_planner go2_nav.launch.py \\
        map_package_dir:=/path/to/octomap_package launch_map_gui:=true
"""

from __future__ import annotations

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_d1_controller_params(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return dict(config.get("d1_controller", {}).get("ros__parameters", {}))


def load_nav_file_config(config_path: str) -> tuple[str, str]:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    relocalization_pcd_file = os.path.abspath(
        os.path.expanduser(str(config.get("relocalization_pcd_file", "")).strip())
    )
    map_package_dir = os.path.abspath(
        os.path.expanduser(str(config.get("map_package_dir", "")).strip())
    )
    return relocalization_pcd_file, map_package_dir


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    dong_octomap_share = get_package_share_directory("dong_octomap")
    nav_params_config = os.path.join(octo_planner_share, "config", "nav_params_go2.yaml")
    _, default_map_package_dir = load_nav_file_config(nav_params_config)
    d1_controller_params = load_d1_controller_params(nav_params_config)

    launch_localization_arg = DeclareLaunchArgument(
        "launch_localization",
        default_value="false",
        description="true=同时启动 Fast-LIO2 重定位",
    )
    prior_pcd_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="/home/allen/maps/scans_0.5m.pcd",
    )
    map_package_arg = DeclareLaunchArgument(
        "map_package_dir",
        default_value=default_map_package_dir,
        description="OctoMap 地图包目录（map_package_manager 自动加载）",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="false",
        description="启动 map_viewer_gui + map_save_gui（Qt 地图管理/导航）",
    )
    launch_planner_arg = DeclareLaunchArgument("launch_planner", default_value="true")
    launch_controller_arg = DeclareLaunchArgument("launch_controller", default_value="true")
    launch_web_arg = DeclareLaunchArgument("launch_web", default_value="true")
    launch_rosbridge_arg = DeclareLaunchArgument("launch_rosbridge", default_value="true")
    web_http_port_arg = DeclareLaunchArgument("web_http_port", default_value="8080")

    fl2_launch = os.path.join(dong_octomap_share, "launch", "fastlio2_localization.launch.py")
    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_launch),
        launch_arguments={
            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
            "locate_in_prior_map": "true",
            "launch_rviz": "false",
            "publish_base_tf": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_localization")),
    )

    planner_params = {
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
        "max_iterations": 500000,
        "snap_search_radius_cells": 13,
        "require_ground_support": True,
        "strict_direct_ground_support": False,
        "enable_preblocked_costmap": True,
        "preblocked_costmap_radius_cells": 3,
        "preblocked_costmap_weight": 2.5,
    }

    planner_node = Node(
        package="octo_planner",
        executable="dong_path_node",
        name="dong_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[planner_params],
    )

    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        parameters=[
            d1_controller_params,
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "tracking_point_marker_topic": "/tracking_point_marker",
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
        parameters=[{"autoload_package_path": LaunchConfiguration("map_package_dir")}],
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

    map_viewer_gui_node = Node(
        package="dong_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
        additional_env={"MAP_VIEWER_DEFAULT_PACKAGE": LaunchConfiguration("map_package_dir")},
        parameters=[
            {
                "tf_parent_frame": "map",
                "tf_child_frame": "base_link",
            }
        ],
    )

    map_save_gui_node = Node(
        package="dong_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    web_root = os.path.join(dong_octomap_share, "web")
    web_server_script = os.path.abspath(
        os.path.join(dong_octomap_share, "..", "..", "lib", "dong_octomap", "no_cache_http_server.py")
    )

    web_http_server = ExecuteProcess(
        cmd=[web_server_script, "--port", LaunchConfiguration("web_http_port"), "--directory", web_root],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_web")),
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
    )

    return LaunchDescription(
        [
            launch_localization_arg,
            prior_pcd_arg,
            map_package_arg,
            launch_map_gui_arg,
            launch_planner_arg,
            launch_controller_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            web_http_port_arg,
            localization_include,
            map_package_manager_node,
            occupied_marker_node,
            planner_node,
            controller_node,
            map_viewer_gui_node,
            map_save_gui_node,
            web_http_server,
            rosbridge_node,
        ]
    )
