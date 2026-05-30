"""走廊 + MPPI：global_path_manager + Nav2 MPPI + path follower。

统一 /costmap/costmap；MPPI 内嵌 costmap 参数与 local_costmap_standalone.yaml 一致。
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav_stack")
    corridor_cfg = os.path.join(pkg, "config", "corridor_mppi.yaml")
    mppi_params = os.path.join(pkg, "config", "nav2_mppi_params.yaml")
    costmap_params = os.path.join(pkg, "config", "local_costmap_standalone.yaml")
    bringup_cfg = os.path.join(pkg, "config", "nav2_controller_bringup.yaml")
    local_display_cfg = os.path.join(pkg, "config", "local_path_display.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # 里程计桥接须早于 controller 激活，否则 MPPI 无 /nav2_odom 不发 cmd_vel
            Node(
                package="go2_nav_stack",
                executable="nav2_odom_bridge",
                name="nav2_odom_bridge",
                output="screen",
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="costmap",
                namespace="costmap",
                output="screen",
                parameters=[costmap_params, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("costmap", "/costmap/costmap"),
                    ("costmap_updates", "/costmap/costmap_updates"),
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[mppi_params, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cmd_vel", "/cmd_vel"),
                    ("odom", "/nav2_odom"),
                ],
            ),
            TimerAction(
                period=8.0,
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
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="go2_nav_stack",
                        executable="mppi_path_follower",
                        output="screen",
                        parameters=[corridor_cfg],
                    ),
                    Node(
                        package="go2_nav_stack",
                        executable="local_path_display",
                        output="screen",
                        parameters=[local_display_cfg],
                    ),
                ],
            ),
        ]
    )
