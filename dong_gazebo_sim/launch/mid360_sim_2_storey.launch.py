"""2_storey：**Go2 四足真狗** + Mid-360 + FAST-LIO2 + RViz（默认）。

平面方块请用 ``mid360_sim_planar_fastlio2.launch.py``。

用法::

    source install/setup.bash
    ros2 launch dong_gazebo_sim mid360_sim_2_storey.launch.py \\
        launch_rviz:=true \\
        prior_pcd_path:=/home/allen/maps/2_storey/global_map_gazebo.pcd
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    octo_share = get_package_share_directory("dong_octomap")
    map_pkg = "/home/allen/maps/2_storey"
    prior_pcd = os.path.join(map_pkg, "global_map_gazebo.pcd")
    if not os.path.isfile(prior_pcd):
        prior_pcd = os.path.join(map_pkg, "global_map.pcd")
    world = os.path.join(octo_share, "worlds", "2_storey.world")

    legged_fl2 = os.path.join(sim_share, "launch", "mid360_sim_legged_fastlio2.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=world),
            DeclareLaunchArgument("prior_pcd_path", default_value=prior_pcd),
            DeclareLaunchArgument("initial_x", default_value="1.0"),
            DeclareLaunchArgument("initial_y", default_value="-2.5"),
            DeclareLaunchArgument("initial_z", default_value="0.275"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="20.0"),
            DeclareLaunchArgument("icp_planar_constraint", default_value="false"),
            DeclareLaunchArgument("champ_locomotion", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(legged_fl2),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "gui": LaunchConfiguration("gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                    "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
                    "champ_locomotion": LaunchConfiguration("champ_locomotion"),
                }.items(),
            ),
        ]
    )
