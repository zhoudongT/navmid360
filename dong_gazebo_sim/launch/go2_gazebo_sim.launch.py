"""Go2 Gazebo Classic：spawn → 站立关节 → 加载控制器 →（由 bringup 再启 CHAMP）。"""

from __future__ import annotations

import os

import launch_ros
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )
    go2_cfg = launch_ros.substitutions.FindPackageShare(package="champ_config").find(
        "champ_config"
    )
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    description_path = LaunchConfiguration("description_path")
    spawn_delay = LaunchConfiguration("spawn_delay_sec")

    gazebo_config = os.path.join(gz_share, "config/gazebo.yaml")
    links_config = os.path.join(go2_cfg, "config/links/links.yaml")

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go2")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_gui = DeclareLaunchArgument("gui", default_value="true")
    declare_world = DeclareLaunchArgument("world")
    declare_description = DeclareLaunchArgument("description_path")
    declare_spawn_delay = DeclareLaunchArgument(
        "spawn_delay_sec",
        default_value="5.0",
        description="gzserver 就绪后延时 spawn",
    )
    declare_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_z = DeclareLaunchArgument(
        "world_init_z",
        default_value="0.275",
        description="与 champ_config Gazebo spawn 一致",
    )
    declare_yaw = DeclareLaunchArgument("world_init_heading", default_value="0.0")
    declare_contact = DeclareLaunchArgument(
        "use_gazebo_contact_sensor",
        default_value="false",
        description="true=champ_gazebo contact_sensor；false=用 sim_foot_contacts_stub",
    )

    urdf_topic_pub = Node(
        package="go2_nav_stack",
        executable="robot_description_topic_publisher",
        name="robot_description_topic_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"xacro_path": description_path},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": Command(["xacro ", description_path])},
            {"publish_frequency": 50.0},
        ],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzserver.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "params_file": gazebo_config,
            "factory": "true",
            "init": "true",
            "force_system": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzclient.launch.py"])
        ),
        condition=IfCondition(gui),
    )

    wait_spawn = Node(
        package="go2_nav_stack",
        executable="wait_for_gazebo_spawn",
        name="wait_for_gazebo_spawn",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"timeout_sec": 120.0},
            {"service_name": "/spawn_entity"},
        ],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "/robot_description",
            "-timeout",
            "60",
            "-package_to_model",
            "-unpause",
            "-x",
            LaunchConfiguration("world_init_x"),
            "-y",
            LaunchConfiguration("world_init_y"),
            "-z",
            LaunchConfiguration("world_init_z"),
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            LaunchConfiguration("world_init_heading"),
        ],
    )

    set_stand = Node(
        package="go2_nav_stack",
        executable="sim_go2_set_stand_pose",
        name="sim_go2_set_stand_pose",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"hold_forever": True},
            {"ramp_sec": 3.0},
            {"hold_hz": 50.0},
        ],
    )

    foot_contacts_stub = Node(
        package="go2_nav_stack",
        executable="sim_foot_contacts_stub",
        name="sim_foot_contacts_stub",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "ros2 control load_controller --set-state active joint_states_controller && "
            "ros2 control load_controller --set-state active joint_group_effort_controller",
        ],
        output="screen",
    )

    contact = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, links_config],
        condition=IfCondition(LaunchConfiguration("use_gazebo_contact_sensor")),
    )

    spawn_after_wait = RegisterEventHandler(
        OnProcessExit(target_action=wait_spawn, on_exit=[spawn])
    )
    ctrl_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[load_controllers, contact])
    )
    stand_after_ctrl = RegisterEventHandler(
        OnProcessExit(target_action=load_controllers, on_exit=[set_stand, foot_contacts_stub])
    )
    delayed_wait = TimerAction(period=spawn_delay, actions=[wait_spawn])

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_gui,
            declare_world,
            declare_description,
            declare_spawn_delay,
            declare_x,
            declare_y,
            declare_z,
            declare_yaw,
            declare_contact,
            urdf_topic_pub,
            robot_state_publisher,
            gzserver,
            gzclient,
            spawn_after_wait,
            ctrl_after_spawn,
            stand_after_ctrl,
            delayed_wait,
        ]
    )
