"""Gazebo Classic + 平面方块 Mid-360 仿真（planar_move + Livox 插件）。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    livox_sim_prefix = get_package_prefix("ros2_livox_simulation")
    models_dir = os.path.join(sim_share, "models")
    xacro_path = os.path.join(sim_share, "urdf", "go2_mid360_sim.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", "'", xacro_path, "'"]),
        value_type=str,
    )
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_share, "worlds", "Building.world"),
        description="Gazebo world",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument(
        "z",
        default_value="0.30",
        description="方块底面贴地时约 0.30（box 高 0.10，中心 z=0.05）",
    )
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="false")
    spawn_delay_arg = DeclareLaunchArgument(
        "spawn_delay_sec",
        default_value="3.0",
        description="gzserver 就绪后延时 spawn",
    )

    set_plugin_path = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(livox_sim_prefix, "lib"),
    )
    set_model_path = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(
            [
                models_dir,
                os.path.join(get_package_share_directory("ros2_livox_simulation"), ".."),
            ]
        ),
    )

    urdf_topic_pub = Node(
        package="go2_nav_stack",
        executable="robot_description_topic_publisher",
        name="robot_description_topic_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"xacro_path": xacro_path},
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
            {"publish_frequency": 50.0},
        ],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzserver.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "factory": "true",
            "init": "true",
            "force_system": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzclient.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    wait_spawn = Node(
        package="go2_nav_stack",
        executable="wait_for_gazebo_spawn",
        name="wait_for_gazebo_spawn",
        output="screen",
        parameters=[{"use_sim_time": True}, {"timeout_sec": 120.0}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-entity",
            "go2_mid360_sim",
            "-topic",
            "/robot_description",
            "-timeout",
            "60",
            "-package_to_model",
            "-unpause",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("dong_octomap"), "rviz", "mid360_loc.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    spawn_after_wait = RegisterEventHandler(
        OnProcessExit(target_action=wait_spawn, on_exit=[spawn])
    )
    delayed_wait = TimerAction(
        period=LaunchConfiguration("spawn_delay_sec"), actions=[wait_spawn]
    )

    return LaunchDescription(
        [
            world_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            gui_arg,
            rviz_arg,
            spawn_delay_arg,
            set_plugin_path,
            set_model_path,
            urdf_topic_pub,
            rsp,
            gzserver,
            gzclient,
            spawn_after_wait,
            delayed_wait,
            rviz,
        ]
    )
