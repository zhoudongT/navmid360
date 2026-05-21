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
            # 可执行文件内部节点名固定为 costmap；name 须为 costmap 才能加载下方 yaml
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="costmap",
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
                    ("odom", "/state_estimation"),
                ],
            ),
            # MPPI 加载较慢；延迟启动 bringup，避免 lifecycle 服务尚未注册
            TimerAction(
                period=28.0,
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
