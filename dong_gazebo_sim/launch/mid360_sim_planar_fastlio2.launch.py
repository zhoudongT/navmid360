"""Gazebo 平面方块 + Mid-360 + FAST-LIO2 + ICP（无 go2_nav_stack / 无四足 CHAMP）。

用法::

    ros2 launch dong_gazebo_sim mid360_sim_planar_fastlio2.launch.py \\
        world:=$(ros2 pkg prefix dong_octomap)/share/dong_octomap/worlds/2_storey.world \\
        prior_pcd_path:=/home/allen/maps/2_storey/global_map_gazebo.pcd \\
        launch_rviz:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    dong_share = get_package_share_directory("dong_octomap")

    gazebo_planar = os.path.join(sim_share, "launch", "gazebo_mid360_bringup.launch.py")
    fl2_wrap = os.path.join(dong_share, "launch", "mid360_fastlio2_localization.launch.py")
    rviz_cfg = os.path.join(dong_share, "rviz", "mid360_loc.rviz")
    icp_sim_cfg = os.path.join(sim_share, "config", "icp_sim_2_storey.yaml")
    fl_planar_cfg = os.path.join(sim_share, "config", "fastlio_planar_sim.yaml")

    octo_share = get_package_share_directory("dong_octomap")
    default_world = os.path.join(octo_share, "worlds", "2_storey.world")
    if not os.path.isfile(default_world):
        default_world = os.path.join(sim_share, "worlds", "2_storey.world")

    seed_initialpose = TimerAction(
        period=LaunchConfiguration("initialpose_delay_sec"),
        actions=[
            Node(
                package="go2_nav_stack",
                executable="sim_initialpose_publisher",
                name="sim_initialpose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "delay_sec": 0.5,
                        "x": LaunchConfiguration("initial_x"),
                        "y": LaunchConfiguration("initial_y"),
                        "z": LaunchConfiguration("initial_z"),
                        "yaw": LaunchConfiguration("initial_a"),
                    }
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration("auto_publish_initialpose")),
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument(
                "prior_pcd_path",
                default_value="/home/allen/maps/2_storey/global_map_gazebo.pcd",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("initial_x", default_value="1.0"),
            DeclareLaunchArgument("initial_y", default_value="-2.5"),
            DeclareLaunchArgument("initial_z", default_value="0.30"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument(
                "auto_publish_initialpose",
                default_value="true",
                description="true：spawn 后自动 /initialpose 触发 ICP",
            ),
            DeclareLaunchArgument(
                "initialpose_delay_sec",
                default_value="14.0",
                description="自 launch 起延时发 /initialpose（秒，应晚于 RViz/spawn）",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_planar),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "launch_rviz": "false",
                    "x": LaunchConfiguration("initial_x"),
                    "y": LaunchConfiguration("initial_y"),
                    "z": LaunchConfiguration("initial_z"),
                    "yaw": LaunchConfiguration("initial_a"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fl2_wrap),
                launch_arguments={
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "locate_in_prior_map": "true",
                    "icp_planar_constraint": "true",
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
                    "rviz_config": rviz_cfg,
                    "launch_livox_driver": "false",
                    "use_sim_time": "true",
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                    "publish_base_tf": "false",
                    "send_odom_base_tf": "true",
                    "enable_transform_publisher": "true",
                    "odom_output_topic": "/Odometry",
                    "prior_map_freeze_odom_without_features": "false",
                    "prior_map_match_dist_sq": "81.0",
                    "prior_map_min_effective_points": "12",
                    "icp_overlay_config": icp_sim_cfg,
                    "fastlio_overlay_config": fl_planar_cfg,
                }.items(),
            ),
            seed_initialpose,
        ]
    )
