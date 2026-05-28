"""CHAMP：仿真默认仅 state_estimation + 持续站立保持；需行走时 champ_locomotion:=true。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    default_gait = os.path.join(
        get_package_share_directory("champ_config"), "config", "gait", "gait.yaml"
    )
    stable_gait = os.path.join(sim_share, "config", "gait_sim_stable.yaml")

    delay_arg = DeclareLaunchArgument(
        "champ_control_delay_sec",
        default_value="16.0",
        description="自 launch 起延时启 CHAMP state_estimation",
    )
    loco_arg = DeclareLaunchArgument(
        "champ_locomotion",
        default_value="false",
        description="true=启 quadruped 步态（会与站立保持冲突，仅行走测试时用）",
    )
    desc_arg = DeclareLaunchArgument("description_path")
    joints_arg = DeclareLaunchArgument("joints_map_path")
    links_arg = DeclareLaunchArgument("links_map_path")
    gait_arg = DeclareLaunchArgument(
        "gait_config_path",
        default_value=stable_gait if os.path.isfile(stable_gait) else default_gait,
    )

    loco_on = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("champ_locomotion"),
                "' in ('true', '1', 'yes')",
            ]
        )
    )

    common_params = [
        {"use_sim_time": True},
        {"gazebo": True},
        {
            "urdf": Command(["xacro ", LaunchConfiguration("description_path")]),
        },
        LaunchConfiguration("joints_map_path"),
        LaunchConfiguration("links_map_path"),
        LaunchConfiguration("gait_config_path"),
    ]

    quadruped = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        name="quadruped_controller_node",
        output="screen",
        parameters=[
            *common_params,
            {"publish_joint_states": False},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {
                "joint_controller_topic": (
                    "joint_group_effort_controller/joint_trajectory"
                )
            },
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
        condition=loco_on,
    )

    state_est = Node(
        package="champ_base",
        executable="state_estimation_node",
        name="state_estimation_node",
        output="screen",
        parameters=[
            *common_params,
            {"orientation_from_imu": True},
        ],
    )

    delayed = TimerAction(
        period=LaunchConfiguration("champ_control_delay_sec"),
        actions=[state_est, quadruped],
    )

    return LaunchDescription(
        [
            delay_arg,
            loco_arg,
            desc_arg,
            joints_arg,
            links_arg,
            gait_arg,
            delayed,
        ]
    )
