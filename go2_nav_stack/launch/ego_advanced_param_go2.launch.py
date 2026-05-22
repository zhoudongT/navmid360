"""Go2 定制 ego_planner_node 参数（map 系 + 点云避障）。

核心数值见 config/ego_planner_core.yaml；max_vel / planning_horizon 可由上层 launch 覆盖。
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory("go2_nav_stack")
    ego_core_yaml = os.path.join(go2_share, "config", "ego_planner_core.yaml")

    map_size_x = LaunchConfiguration("map_size_x_", default="40.0")
    map_size_y = LaunchConfiguration("map_size_y_", default="40.0")
    map_size_z = LaunchConfiguration("map_size_z_", default="2.5")
    odometry_topic = LaunchConfiguration("odometry_topic", default="state_estimation")
    cloud_topic = LaunchConfiguration("cloud_topic", default="cloud_registered_map")
    max_vel = LaunchConfiguration("max_vel", default="0.35")
    max_acc = LaunchConfiguration("max_acc", default="2.0")
    planning_horizon = LaunchConfiguration("planning_horizon", default="6.0")
    flight_type = LaunchConfiguration("flight_type", default="1")
    drone_id = LaunchConfiguration("drone_id", default="0")

    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name=["drone_", drone_id, "_ego_planner_node"],
        output="screen",
        remappings=[
            ("odom_world", ["drone_", drone_id, "_", odometry_topic]),
            ("planning/bspline", ["drone_", drone_id, "_planning/bspline"]),
            ("planning/data_display", ["drone_", drone_id, "_planning/data_display"]),
            ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
            ("goal_point", ["drone_", drone_id, "_plan_vis/goal_point"]),
            ("global_list", ["drone_", drone_id, "_plan_vis/global_list"]),
            ("init_list", ["drone_", drone_id, "_plan_vis/init_list"]),
            ("optimal_list", ["drone_", drone_id, "_plan_vis/optimal_list"]),
            ("a_star_list", ["drone_", drone_id, "_plan_vis/a_star_list"]),
            ("grid_map/odom", ["drone_", drone_id, "_", odometry_topic]),
            ("grid_map/cloud", ["drone_", drone_id, "_", cloud_topic]),
            ("grid_map/occupancy_inflate", ["drone_", drone_id, "_grid/grid_map/occupancy_inflate"]),
        ],
        parameters=[
            ego_core_yaml,
            {
                "fsm/flight_type": flight_type,
                "fsm/planning_horizon": planning_horizon,
                "grid_map/map_size_x": map_size_x,
                "grid_map/map_size_y": map_size_y,
                "grid_map/map_size_z": map_size_z,
                "manager/max_vel": max_vel,
                "manager/max_acc": max_acc,
                "manager/planning_horizon": planning_horizon,
                "manager/drone_id": drone_id,
                "optimization/max_vel": max_vel,
                "optimization/max_acc": max_acc,
                "bspline/limit_vel": max_vel,
                "bspline/limit_acc": max_acc,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_size_x_", default_value=map_size_x),
            DeclareLaunchArgument("map_size_y_", default_value=map_size_y),
            DeclareLaunchArgument("map_size_z_", default_value=map_size_z),
            DeclareLaunchArgument("odometry_topic", default_value=odometry_topic),
            DeclareLaunchArgument("cloud_topic", default_value=cloud_topic),
            DeclareLaunchArgument("max_vel", default_value=max_vel),
            DeclareLaunchArgument("max_acc", default_value=max_acc),
            DeclareLaunchArgument("planning_horizon", default_value=planning_horizon),
            DeclareLaunchArgument("flight_type", default_value=flight_type),
            DeclareLaunchArgument("drone_id", default_value=drone_id),
            ego_planner_node,
        ]
    )
