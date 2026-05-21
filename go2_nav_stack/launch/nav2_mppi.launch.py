"""Nav2：独立 local_costmap（RViz 显示）+ controller_server (MPPI) + bringup。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav_stack")
    mppi_params = os.path.join(pkg, "config", "nav2_mppi_params.yaml")
    costmap_params = os.path.join(pkg, "config", "local_costmap_standalone.yaml")
    bringup_cfg = os.path.join(pkg, "config", "nav2_controller_bringup.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="local_costmap",
                output="screen",
                parameters=[costmap_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[mppi_params, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cmd_vel", "/cmd_vel"),
                    ("odom", "/state_estimation"),
                ],
            ),
            # 等 controller_server / costmap 进程注册后再 lifecycle activate
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="go2_nav_stack",
                        executable="nav2_controller_bringup",
                        name="nav2_controller_bringup",
                        output="screen",
                        parameters=[bringup_cfg, {"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
        ]
    )
