"""Gazebo Classic + Go2 四足（CHAMP）+ Mid-360 仿真。

前置::

  bash scripts/sync_go2_champ_gazebo.sh
  bash scripts/build_go2_champ_gazebo.sh

话题（与 mid360 平面栈一致）:
  /livox/lidar   livox_ros_driver2/msg/CustomMsg
  /livox/imu     sensor_msgs/msg/Imu  （由 /imu/data relay）
  /cmd_vel       geometry_msgs/msg/Twist  → CHAMP 步态
  /odom          nav_msgs/Odometry  （CHAMP 闭环里程计）

用法::

    ros2 launch dong_gazebo_sim gazebo_go2_legged_mid360_bringup.launch.py \\
        world:=$(ros2 pkg prefix dong_octomap)/share/dong_octomap/worlds/2_storey.world
"""

from __future__ import annotations

import os

import launch_ros
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        config_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_config").find(
            "go2_config"
        )
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 go2_config。请先:\n"
            "  bash scripts/sync_go2_champ_gazebo.sh\n"
            "  bash scripts/build_go2_champ_gazebo.sh"
        ) from exc

    sim_share = get_package_share_directory("dong_gazebo_sim")
    livox_sim_prefix = get_package_prefix("ros2_livox_simulation")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    mid360_model_path = os.path.join(descr_pkg_share, "xacro/robot_mid360.xacro")
    if not os.path.isfile(mid360_model_path):
        raise RuntimeError(
            f"缺少 {mid360_model_path}\n"
            "请运行: bash scripts/sync_go2_champ_gazebo.sh"
        )

    octo_share = get_package_share_directory("dong_octomap")
    default_world = os.path.join(octo_share, "worlds", "2_storey.world")
    if not os.path.isfile(default_world):
        default_world = os.path.join(sim_share, "worlds", "2_storey.world")

    models_dir = os.path.join(sim_share, "models")
    go2_gazebo_launch = os.path.join(sim_share, "launch", "go2_gazebo_sim.launch.py")
    champ_control_launch = os.path.join(
        sim_share, "launch", "go2_champ_sim_control.launch.py"
    )

    world_arg = DeclareLaunchArgument("world", default_value=default_world)
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    x_arg = DeclareLaunchArgument("x", default_value="1.0")
    y_arg = DeclareLaunchArgument("y", default_value="-2.5")
    z_arg = DeclareLaunchArgument(
        "z",
        default_value="0.275",
        description="spawn z（略高于 0.275，Mid-360 支架不易蹭地）",
    )
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    spawn_delay_arg = DeclareLaunchArgument(
        "spawn_delay_sec",
        default_value="3.0",
        description="发布 /robot_description 后延时 spawn",
    )
    champ_delay_arg = DeclareLaunchArgument(
        "champ_control_delay_sec",
        default_value="16.0",
        description="延时启 CHAMP state_estimation（不启步态时仍保持站立）",
    )
    loco_arg = DeclareLaunchArgument(
        "champ_locomotion",
        default_value="false",
        description="true=启用 CHAMP 行走（默认 false，仅站立+定位）",
    )
    settle_arg = DeclareLaunchArgument(
        "legged_settle_sec",
        default_value="12.0",
        description="CHAMP 启动后持续发零 /cmd_vel 时长（秒）",
    )

    set_plugin_path = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(livox_sim_prefix, "lib"),
    )
    set_model_path = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(get_package_share_directory("ros2_livox_simulation"), ".."),
    )
    set_local_models = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=models_dir,
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_gazebo_launch),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "go2",
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "description_path": mid360_model_path,
            "world_init_x": LaunchConfiguration("x"),
            "world_init_y": LaunchConfiguration("y"),
            "world_init_z": LaunchConfiguration("z"),
            "world_init_heading": LaunchConfiguration("yaw"),
            "spawn_delay_sec": LaunchConfiguration("spawn_delay_sec"),
        }.items(),
    )

    champ_control_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(champ_control_launch),
        launch_arguments={
            "description_path": mid360_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "champ_control_delay_sec": LaunchConfiguration("champ_control_delay_sec"),
            "champ_locomotion": LaunchConfiguration("champ_locomotion"),
        }.items(),
    )

    imu_relay = Node(
        package="topic_tools",
        executable="relay",
        name="imu_to_livox_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/imu/data", "/livox/imu"],
    )

    legged_settle = TimerAction(
        period=LaunchConfiguration("champ_control_delay_sec"),
        actions=[
            Node(
                package="go2_nav_stack",
                executable="sim_legged_settle",
                name="sim_legged_settle",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "settle_duration_sec": LaunchConfiguration("legged_settle_sec"),
                        "cmd_vel_topic": "/cmd_vel",
                        "publish_rate_hz": 30.0,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            spawn_delay_arg,
            champ_delay_arg,
            loco_arg,
            settle_arg,
            set_plugin_path,
            set_model_path,
            set_local_models,
            gazebo_ld,
            champ_control_ld,
            imu_relay,
            legged_settle,
        ]
    )
