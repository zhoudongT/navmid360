"""Gazebo **Go2 四足真狗** + 狗头 Mid-360 + FAST-LIO2 + ICP + RViz 点云。

前置（首次）::

    bash scripts/sync_go2_champ_gazebo.sh
    bash scripts/build_go2_champ_gazebo.sh

用法::

    source install/setup.bash
    ros2 launch dong_gazebo_sim mid360_sim_legged_fastlio2.launch.py \\
        launch_rviz:=true \\
        prior_pcd_path:=/home/allen/maps/2_storey/global_map_gazebo.pcd
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

    gazebo_legged = os.path.join(sim_share, "launch", "gazebo_go2_legged_mid360_bringup.launch.py")
    fl2_wrap = os.path.join(dong_share, "launch", "mid360_fastlio2_localization.launch.py")
    rviz_cfg = os.path.join(dong_share, "rviz", "mid360_loc.rviz")
    icp_sim_cfg = os.path.join(sim_share, "config", "icp_sim_2_storey.yaml")
    fl_legged_cfg = os.path.join(sim_share, "config", "fastlio_go2_head_sim.yaml")

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
            DeclareLaunchArgument(
                "rviz_delay_sec",
                default_value="20.0",
                description="四足站稳后再开 RViz（秒）",
            ),
            DeclareLaunchArgument("initial_x", default_value="1.0"),
            DeclareLaunchArgument("initial_y", default_value="-2.5"),
            DeclareLaunchArgument("initial_z", default_value="0.275"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument(
                "icp_planar_constraint",
                default_value="false",
                description="2_storey 楼梯场景建议 false",
            ),
            DeclareLaunchArgument("champ_control_delay_sec", default_value="16.0"),
            DeclareLaunchArgument("legged_settle_sec", default_value="12.0"),
            DeclareLaunchArgument("champ_locomotion", default_value="false"),
            DeclareLaunchArgument("auto_publish_initialpose", default_value="true"),
            DeclareLaunchArgument("initialpose_delay_sec", default_value="24.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_legged),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "x": LaunchConfiguration("initial_x"),
                    "y": LaunchConfiguration("initial_y"),
                    "z": LaunchConfiguration("initial_z"),
                    "yaw": LaunchConfiguration("initial_a"),
                    "champ_control_delay_sec": LaunchConfiguration("champ_control_delay_sec"),
                    "legged_settle_sec": LaunchConfiguration("legged_settle_sec"),
                    "champ_locomotion": LaunchConfiguration("champ_locomotion"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fl2_wrap),
                launch_arguments={
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "locate_in_prior_map": "true",
                    "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
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
                    "fastlio_overlay_config": fl_legged_cfg,
                }.items(),
            ),
            seed_initialpose,
        ]
    )
