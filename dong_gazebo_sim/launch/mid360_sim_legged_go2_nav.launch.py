"""Go2 四足真狗 + FAST-LIO2 + go2_nav_stack 全栈（2_storey）。

用法::

    ros2 launch dong_gazebo_sim mid360_sim_legged_go2_nav.launch.py \\
        prior_pcd_path:=/home/allen/maps/2_storey/global_map_gazebo.pcd \\
        launch_rviz:=true reloc_auto_start:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    sim_share = get_package_share_directory("dong_gazebo_sim")
    go2_share = get_package_share_directory("go2_nav_stack")

    legged_sim = os.path.join(sim_share, "launch", "mid360_sim_legged_fastlio2.launch.py")
    full_stack = os.path.join(go2_share, "launch", "go2_full_stack.launch.py")

    map_pkg = "/home/allen/maps/2_storey"
    prior_pcd = os.path.join(map_pkg, "global_map_gazebo.pcd")
    if not os.path.isfile(prior_pcd):
        prior_pcd = os.path.join(map_pkg, "global_map.pcd")

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("prior_pcd_path", default_value=prior_pcd),
            DeclareLaunchArgument("map_package_dir", default_value=map_pkg),
            DeclareLaunchArgument("maps_root", default_value="/home/allen/maps"),
            DeclareLaunchArgument("initial_x", default_value="1.0"),
            DeclareLaunchArgument("initial_y", default_value="-2.5"),
            DeclareLaunchArgument("initial_z", default_value="0.275"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="20.0"),
            DeclareLaunchArgument("reloc_auto_start", default_value="true"),
            DeclareLaunchArgument("nav_stack_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("reloc_startup_delay_sec", default_value="12.0"),
            DeclareLaunchArgument("launch_map_gui", default_value="true"),
            DeclareLaunchArgument("launch_web", default_value="true"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument("web_http_port", default_value="8081"),
            DeclareLaunchArgument("rosbridge_port", default_value="9091"),
            DeclareLaunchArgument("legged_sim", default_value="true"),
            DeclareLaunchArgument("stair_navigation", default_value="true"),
            DeclareLaunchArgument("icp_planar_constraint", default_value="false"),
            DeclareLaunchArgument("champ_locomotion", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(legged_sim),
                launch_arguments={
                    "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                    "gui": LaunchConfiguration("gui"),
                    "launch_rviz": "false",
                    "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                    "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
                    "champ_locomotion": LaunchConfiguration("champ_locomotion"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("nav_stack_delay_sec"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(full_stack),
                        launch_arguments={
                            "maps_root": LaunchConfiguration("maps_root"),
                            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
                            "map_package_dir": LaunchConfiguration("map_package_dir"),
                            "launch_rviz": LaunchConfiguration("launch_rviz"),
                            "launch_map_gui": LaunchConfiguration("launch_map_gui"),
                            "launch_web": LaunchConfiguration("launch_web"),
                            "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
                            "web_http_port": LaunchConfiguration("web_http_port"),
                            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
                            "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
                            "reloc_startup_delay_sec": LaunchConfiguration(
                                "reloc_startup_delay_sec"
                            ),
                            "launch_fastlio": "false",
                            "use_sim_time": "true",
                            "legged_sim": LaunchConfiguration("legged_sim"),
                            "stair_navigation": LaunchConfiguration("stair_navigation"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
