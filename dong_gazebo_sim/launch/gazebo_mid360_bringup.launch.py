"""启动 Gazebo Classic + Go2 简化模型 + Mid-360 仿真激光 + /livox/imu。

话题（与实车一致）:
  /livox/lidar   livox_ros_driver2/msg/CustomMsg
  /livox/imu     sensor_msgs/msg/Imu
  /cmd_vel       geometry_msgs/msg/Twist  （平面运动插件）

用法::

    source install/setup.bash
    ros2 launch dong_gazebo_sim gazebo_mid360_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    livox_sim_prefix = get_package_prefix("ros2_livox_simulation")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_share, "worlds", "sim_playground.world"),
        description="Gazebo world 文件绝对路径",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.35")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="false")

    xacro_path = os.path.join(sim_share, "urdf", "go2_mid360_sim.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", "'", xacro_path, "'"]),
        value_type=str,
    )

    set_plugin_path = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(livox_sim_prefix, "lib"),
    )
    set_model_path = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(get_package_share_directory("ros2_livox_simulation"), ".."),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "verbose": "false",
        }.items(),
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "go2_mid360_sim",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
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

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(yaw_arg)
    ld.add_action(gui_arg)
    ld.add_action(rviz_arg)
    ld.add_action(set_plugin_path)
    ld.add_action(set_model_path)
    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(spawn)
    ld.add_action(rviz)
    return ld
