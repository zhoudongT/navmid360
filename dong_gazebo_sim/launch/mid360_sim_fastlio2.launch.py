"""Gazebo Mid-360 + FAST-LIO2-Localization + ICP（无实机 Livox 驱动）。

与实车 ``mid360_fastlio2_localization`` / ``bringup_humble`` 对齐：``/lio_odom_raw``、ICP、Scan Context。

用法::

    source install/setup.bash
    ros2 launch dong_gazebo_sim mid360_sim_fastlio2.launch.py \\
        prior_pcd_path:=/home/allen/maps/Building.pcd
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    dong_share = get_package_share_directory("dong_octomap")
    go2_share = get_package_share_directory("go2_nav_stack")

    gazebo_planar = os.path.join(sim_share, "launch", "gazebo_mid360_bringup.launch.py")
    gazebo_legged = os.path.join(sim_share, "launch", "gazebo_go2_legged_mid360_bringup.launch.py")
    fl2_wrap = os.path.join(dong_share, "launch", "mid360_fastlio2_localization.launch.py")
    rviz_go2 = os.path.join(go2_share, "config", "rviz_go2_nav.rviz")
    icp_sim_cfg = os.path.join(sim_share, "config", "icp_sim_2_storey.yaml")
    fl_planar_cfg = os.path.join(sim_share, "config", "fastlio_planar_sim.yaml")
    fl_legged_cfg = os.path.join(sim_share, "config", "fastlio_go2_head_sim.yaml")

    octo_share = get_package_share_directory("dong_octomap")
    default_world = os.path.join(octo_share, "worlds", "2_storey.world")
    if not os.path.isfile(default_world):
        default_world = os.path.join(sim_share, "worlds", "2_storey.world")

    world_arg = DeclareLaunchArgument("world", default_value=default_world)
    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="/home/allen/maps/2_storey/global_map.pcd",
    )
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="false")
    rviz_delay_arg = DeclareLaunchArgument(
        "rviz_delay_sec",
        default_value="20.0",
        description="Gazebo 四足站稳后再开 RViz（秒）",
    )
    ix_arg = DeclareLaunchArgument("initial_x", default_value="1.0")
    iy_arg = DeclareLaunchArgument("initial_y", default_value="-2.5")
    iz_arg = DeclareLaunchArgument(
        "initial_z",
        default_value="0.30",
        description="planar 方块 0.30；legged 请传 0.275",
    )
    ia_arg = DeclareLaunchArgument("initial_a", default_value="0.0")
    seed_pose_arg = DeclareLaunchArgument("auto_publish_initialpose", default_value="true")
    reloc_auto_arg = DeclareLaunchArgument("reloc_auto_start", default_value="true")
    initialpose_delay_arg = DeclareLaunchArgument(
        "initialpose_delay_sec",
        default_value="23.0",
        description="RViz 打开后再发 /initialpose（秒，自 launch 起）；reloc_auto_start 时由 ScanContext 负责",
    )
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="legged",
        description="legged=Go2 四足真狗；planar=简化平面方块",
    )
    icp_planar_arg = DeclareLaunchArgument(
        "icp_planar_constraint",
        default_value="true",
        description="legged 楼梯场景建议 false",
    )
    champ_delay_arg = DeclareLaunchArgument(
        "champ_control_delay_sec",
        default_value="12.0",
        description="四足：spawn 后延时再启 CHAMP（秒）",
    )
    legged_settle_arg = DeclareLaunchArgument(
        "legged_settle_sec",
        default_value="12.0",
        description="CHAMP 启动后零速时长（champ_locomotion:=true 时有效）",
    )
    loco_arg = DeclareLaunchArgument(
        "champ_locomotion",
        default_value="false",
        description="false=仅站立保持+定位；true=启用 CHAMP 行走",
    )

    gazebo_planar_ld = IncludeLaunchDescription(
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
        condition=LaunchConfigurationEquals("robot_model", "planar"),
    )
    gazebo_legged_ld = IncludeLaunchDescription(
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
        condition=LaunchConfigurationEquals("robot_model", "legged"),
    )

    _fl2_args = {
        "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
        "locate_in_prior_map": "true",
        "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
        "launch_rviz": LaunchConfiguration("launch_rviz"),
        "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
        "rviz_config": rviz_go2,
        "launch_livox_driver": "false",
        "use_sim_time": "true",
        "initial_x": LaunchConfiguration("initial_x"),
        "initial_y": LaunchConfiguration("initial_y"),
        "initial_z": LaunchConfiguration("initial_z"),
        "initial_a": LaunchConfiguration("initial_a"),
        "publish_base_tf": "false",
        "send_odom_base_tf": "false",
        "enable_transform_publisher": "false",
        "odom_output_topic": "/lio_odom_raw",
        "prior_map_freeze_odom_without_features": "false",
        "prior_map_match_dist_sq": "81.0",
        "prior_map_min_effective_points": "12",
        "icp_overlay_config": icp_sim_cfg,
    }

    fl2_planar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_wrap),
        launch_arguments={
            **_fl2_args,
            "fastlio_overlay_config": fl_planar_cfg,
        }.items(),
        condition=LaunchConfigurationEquals("robot_model", "planar"),
    )
    fl2_legged = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_wrap),
        launch_arguments={
            **_fl2_args,
            "fastlio_overlay_config": fl_legged_cfg,
        }.items(),
        condition=LaunchConfigurationEquals("robot_model", "legged"),
    )

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
                        "delay_sec": 0.5,
                        "x": LaunchConfiguration("initial_x"),
                        "y": LaunchConfiguration("initial_y"),
                        "z": LaunchConfiguration("initial_z"),
                        "yaw": LaunchConfiguration("initial_a"),
                    }
                ],
            )
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("auto_publish_initialpose"),
                    "' in ('true', '1', 'yes') and '",
                    LaunchConfiguration("reloc_auto_start"),
                    "' not in ('true', '1', 'yes')",
                ]
            )
        ),
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter(name="use_sim_time", value=True))
    ld.add_action(world_arg)
    ld.add_action(prior_arg)
    ld.add_action(gui_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_delay_arg)
    ld.add_action(ix_arg)
    ld.add_action(iy_arg)
    ld.add_action(iz_arg)
    ld.add_action(ia_arg)
    ld.add_action(seed_pose_arg)
    ld.add_action(reloc_auto_arg)
    ld.add_action(initialpose_delay_arg)
    ld.add_action(robot_model_arg)
    ld.add_action(icp_planar_arg)
    ld.add_action(champ_delay_arg)
    ld.add_action(legged_settle_arg)
    ld.add_action(loco_arg)
    ld.add_action(gazebo_planar_ld)
    ld.add_action(gazebo_legged_ld)
    ld.add_action(fl2_planar)
    ld.add_action(fl2_legged)
    ld.add_action(seed_initialpose)
    return ld
