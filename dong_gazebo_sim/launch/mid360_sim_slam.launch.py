"""Gazebo Mid-360 仿真 + FAST-LIO 建图（不启动真实 Livox 驱动）。

依赖: fast_lio、ros2_livox_simulation、dong_gazebo_sim 已 colcon build。

用法::

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch dong_gazebo_sim mid360_sim_slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    try:
        fast_lio_share = get_package_share_directory("fast_lio")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 fast_lio。请先编译 Fast-LIO2-Localization/FAST_LIO 或 Ericsii FAST_LIO_ROS2。"
        ) from exc

    sim_share = get_package_share_directory("dong_gazebo_sim")
    dong_octomap_share = get_package_share_directory("dong_octomap")

    fastlio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    gazebo_launch = os.path.join(sim_share, "launch", "gazebo_mid360_bringup.launch.py")

    config_file_arg = DeclareLaunchArgument(
        "fastlio_config_file",
        default_value="mid360_go2_up.yaml",
        description="dong_octomap/config 下的 FAST-LIO 配置",
    )
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "launch_rviz": "false",
        }.items(),
    )

    octomap_cfg = os.path.join(dong_octomap_share, "config")
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "rviz": LaunchConfiguration("launch_rviz"),
            "config_path": octomap_cfg,
            "config_file": LaunchConfiguration("fastlio_config_file"),
        }.items(),
    )

    save_gui = Node(
        package="dong_octomap",
        executable="save_bin_map_gui.py",
        name="save_bin_map_gui",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter(name="use_sim_time", value=True))
    ld.add_action(config_file_arg)
    ld.add_action(gui_arg)
    ld.add_action(rviz_arg)
    ld.add_action(gazebo)
    ld.add_action(fastlio)
    ld.add_action(save_gui)
    return ld
