"""Go2 全局规划栈：OctoMap 规划 + 网页 +（可选）Qt 地图管理器。

局部跟踪与避障由 go2_nav_stack（Nav2 MPPI FollowPath）提供，勿在本 launch 启动 d1_controller。

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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    from ament_index_python.packages import get_package_prefix

    dong_prefix = get_package_prefix("dong_octomap")
    nav_params_config = os.path.join(octo_planner_share, "config", "nav_params_go2.yaml")
    _, default_map_package_dir = load_nav_file_config(nav_params_config)
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
    launch_web_arg = DeclareLaunchArgument("launch_web", default_value="true")
    launch_rosbridge_arg = DeclareLaunchArgument("launch_rosbridge", default_value="true")
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="rosbridge WebSocket 端口；须与 dong_octomap/web/main.js 一致",
    )
    rosbridge_delay_sec_arg = DeclareLaunchArgument(
        "rosbridge_delay_sec",
        default_value="4.0",
        description="rosbridge 延迟启动秒数（FAST_START=1 时 oneclick 可设为 2）",
    )
    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",
        default_value="8888",
        description="静态网页 HTTP 端口（WebSocket/rosbridge 固定 9090）",
    )

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
        "lowest_traversable_only": False,
        "enable_preblocked_costmap": True,
        "preblocked_costmap_radius_cells": 3,
        "preblocked_costmap_weight": 2.5,
        # 全局路径使用 OctoMap 栅格真实 z，不再额外抬高（与机体/地面 z=0 一致）
        "path_z_offset_m": 0.0,
    }

    planner_node = Node(
        package="octo_planner",
        executable="dong_path_node",
        name="dong_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[planner_params],
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
    web_server_script = os.path.join(
        dong_prefix, "lib", "dong_octomap", "no_cache_http_server.py"
    )
    if not os.path.isfile(web_server_script):
        raise RuntimeError(f"未找到网页服务脚本: {web_server_script}")

    web_http_server = ExecuteProcess(
        cmd=[web_server_script, "--port", LaunchConfiguration("web_http_port"), "--directory", web_root],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_web")),
    )

    # rosbridge：优先 install 内脚本，其次仓库 scripts/
    rosbridge_safe = os.path.join(octo_planner_share, "scripts", "run_rosbridge_safe.sh")
    if not os.path.isfile(rosbridge_safe):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
        rosbridge_safe = os.path.join(repo_root, "scripts", "run_rosbridge_safe.sh")
    if not os.path.isfile(rosbridge_safe):
        nav_root = os.environ.get("NAVMID360_ROOT", "/home/allen/navmid360")
        rosbridge_safe = os.path.join(nav_root, "scripts", "run_rosbridge_safe.sh")
    if not os.path.isfile(rosbridge_safe):
        raise RuntimeError(f"未找到 run_rosbridge_safe.sh: {rosbridge_safe}")
    rosbridge_delayed = TimerAction(
        period=LaunchConfiguration("rosbridge_delay_sec"),
        actions=[
            ExecuteProcess(
                cmd=[rosbridge_safe, LaunchConfiguration("rosbridge_port")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
            ),
        ],
    )

    return LaunchDescription(
        [
            launch_localization_arg,
            prior_pcd_arg,
            map_package_arg,
            launch_map_gui_arg,
            launch_planner_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            rosbridge_port_arg,
            rosbridge_delay_sec_arg,
            web_http_port_arg,
            localization_include,
            map_package_manager_node,
            occupied_marker_node,
            planner_node,
            map_viewer_gui_node,
            map_save_gui_node,
            web_http_server,
            rosbridge_delayed,
        ]
    )
