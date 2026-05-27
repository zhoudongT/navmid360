"""平面方块 Gazebo + FAST-LIO2 + 导航全栈（推荐用于 2_storey 导航调试）。

运动：``planar_move`` 订阅 ``/cmd_vel``，发布 ``/gazebo_odom``。
定位：``/lio_odom_raw`` + ``tf_manager_sim.yaml``（航向融合 gazebo_odom）。

用法::

    source install/setup.bash
    ros2 launch dong_gazebo_sim mid360_sim_planar_go2_nav.launch.py \\
        prior_pcd_path:=/home/allen/maps/2_storey/global_map_gazebo.pcd \\
        launch_rviz:=true reloc_auto_start:=true

流程：spawn 方块 → 约 8s RViz → 重定位 → 地图 GUI 规划路径 →「开始导航」。
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    go2_share = get_package_share_directory("go2_nav_stack")

    planar_sim = os.path.join(sim_share, "launch", "mid360_sim_planar_fastlio2.launch.py")
    full_stack = os.path.join(go2_share, "launch", "go2_full_stack.launch.py")

    octo_share = get_package_share_directory("dong_octomap")
    default_world = os.path.join(octo_share, "worlds", "2_storey.world")
    map_pkg = "/home/allen/maps/2_storey"
    prior_pcd = os.path.join(map_pkg, "global_map_gazebo.pcd")
    if not os.path.isfile(prior_pcd):
        prior_pcd = os.path.join(map_pkg, "global_map.pcd")

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("prior_pcd_path", default_value=prior_pcd),
            DeclareLaunchArgument("map_package_dir", default_value=map_pkg),
            DeclareLaunchArgument("maps_root", default_value="/home/allen/maps"),
            DeclareLaunchArgument("initial_x", default_value="1.0"),
            DeclareLaunchArgument("initial_y", default_value="-2.5"),
            DeclareLaunchArgument("initial_z", default_value="0.30"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("reloc_auto_start", default_value="true"),
            DeclareLaunchArgument("nav_stack_delay_sec", default_value="6.0"),
            DeclareLaunchArgument("reloc_startup_delay_sec", default_value="10.0"),
            DeclareLaunchArgument("launch_map_gui", default_value="true"),
            DeclareLaunchArgument("launch_web", default_value="true"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument("web_http_port", default_value="8080"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("scan_context_db", default_value="scan_context_db.json"),
            DeclareLaunchArgument("strict_global_path", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planar_sim),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "gui": LaunchConfiguration("gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                    "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("nav_stack_delay_sec"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(full_stack),
                        launch_arguments={
                            "maps_root": LaunchConfiguration("maps_root"),
                            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                            "map_package_dir": LaunchConfiguration("map_package_dir"),
                            "launch_rviz": "false",
                            "launch_map_gui": LaunchConfiguration("launch_map_gui"),
                            "launch_web": LaunchConfiguration("launch_web"),
                            "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
                            "web_http_port": LaunchConfiguration("web_http_port"),
                            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
                            "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
                            "reloc_startup_delay_sec": LaunchConfiguration(
                                "reloc_startup_delay_sec"
                            ),
                            "scan_context_db": LaunchConfiguration("scan_context_db"),
                            "launch_fastlio": "false",
                            "use_sim_time": "true",
                            "legged_sim": "false",
                            "stair_navigation": "false",
                            "strict_global_path": LaunchConfiguration("strict_global_path"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
