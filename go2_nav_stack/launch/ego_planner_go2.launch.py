"""Go2 实机：ZJU ego-planner-swarm（无仿真器）。

前置::

  bash scripts/sync_ego_planner_swarm.sh
  bash scripts/build_ego_planner_swarm_humble.sh
  source install/setup.bash
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    try:
        get_package_share_directory("ego_planner")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ego_planner。请先:\n"
            "  bash scripts/sync_ego_planner_swarm.sh\n"
            "  bash scripts/build_ego_planner_swarm_humble.sh\n"
            "  source install/setup.bash"
        ) from exc

    go2_share = get_package_share_directory("go2_nav_stack")
    ego_cfg = os.path.join(go2_share, "config", "ego_planner_go2.yaml")
    adv_launch = os.path.join(go2_share, "launch", "ego_advanced_param_go2.launch.py")

    ego_adv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adv_launch),
        launch_arguments={
            "drone_id": LaunchConfiguration("drone_id"),
            "map_size_x_": LaunchConfiguration("map_size_x"),
            "map_size_y_": LaunchConfiguration("map_size_y"),
            "map_size_z_": LaunchConfiguration("map_size_z"),
            "max_vel": LaunchConfiguration("max_vel"),
            "max_acc": LaunchConfiguration("max_acc"),
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "flight_type": "1",
        }.items(),
    )

    traj_server = Node(
        package="ego_planner",
        executable="traj_server",
        name="drone_0_traj_server",
        output="screen",
        remappings=[
            ("position_cmd", "/drone_0_planning/pos_cmd"),
            ("planning/bspline", "/drone_0_planning/bspline"),
        ],
        parameters=[{"traj_server/time_forward": 1.0}],
    )

    return [
        ego_adv,
        traj_server,
        Node(
            package="go2_nav_stack",
            executable="ego_go2_bridge",
            output="screen",
            parameters=[ego_cfg],
        ),
        Node(
            package="go2_nav_stack",
            executable="ego_path_follower",
            output="screen",
            parameters=[ego_cfg],
        ),
        Node(
            package="go2_nav_stack",
            executable="ego_cmd_bridge",
            output="screen",
            parameters=[ego_cfg],
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package="go2_nav_stack",
            executable="ego_traj_display",
            output="screen",
            parameters=[ego_cfg],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_id", default_value="0"),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="2.5"),
            DeclareLaunchArgument("max_vel", default_value="0.35"),
            DeclareLaunchArgument("max_acc", default_value="2.0"),
            DeclareLaunchArgument("planning_horizon", default_value="0.8"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
