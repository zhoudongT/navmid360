"""Gazebo Mid-360 + FAST-LIO2 + go2_nav_stack（可选平面 / 四足）。

**推荐导航调试**：``mid360_sim_2_storey.launch.py`` 或 ``mid360_sim_planar_go2_nav.launch.py``（平面方块，无 CHAMP）。

本 launch 通过 ``robot_model`` 切换：
- ``planar``（默认）：``gazebo_mid360_bringup`` + ``fastlio_planar_sim.yaml``
- ``legged``：四足 URDF + CHAMP（仅单独测四足时用）

用法（四足，不推荐与 2_storey 导航混用）::

    ros2 launch dong_gazebo_sim mid360_sim_go2_nav.launch.py \\
        robot_model:=legged legged_sim:=true ...
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

    fl2_sim = os.path.join(sim_share, "launch", "mid360_sim_fastlio2.launch.py")
    full_stack = os.path.join(go2_share, "launch", "go2_full_stack.launch.py")

    octo_share = get_package_share_directory("dong_octomap")
    default_world = os.path.join(octo_share, "worlds", "2_storey.world")
    if not os.path.isfile(default_world):
        default_world = os.path.join(sim_share, "worlds", "2_storey.world")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world（如 2_storey.world）",
    )
    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="/home/allen/maps/2_storey/global_map.pcd",
        description="先验 PCD；可用 export_octomap_package_to_pcd.py 从 map 包导出",
    )
    map_pkg_arg = DeclareLaunchArgument(
        "map_package_dir",
        default_value="/home/allen/maps/2_storey",
        description="OctoMap 地图包目录（含 meta.yaml layers.npz）",
    )
    maps_root_arg = DeclareLaunchArgument("maps_root", default_value="/home/allen/maps")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="启动 rviz_go2_nav.rviz（与 Gazebo 并排看重定位/路径）",
    )
    rviz_delay_arg = DeclareLaunchArgument(
        "rviz_delay_sec",
        default_value="8.0",
        description="平面方块约 8s；四足建议 20",
    )
    map_gui_arg = DeclareLaunchArgument("launch_map_gui", default_value="true")
    web_arg = DeclareLaunchArgument("launch_web", default_value="true")
    bridge_arg = DeclareLaunchArgument("launch_rosbridge", default_value="true")
    web_port_arg = DeclareLaunchArgument("web_http_port", default_value="8080")
    sc_db_arg = DeclareLaunchArgument(
        "scan_context_db",
        default_value="scan_context_db.json",
        description="相对 maps_root 的 Scan Context 库",
    )
    reloc_arg = DeclareLaunchArgument(
        "reloc_auto_start",
        default_value="true",
        description="仿真 spawn 位姿与 initial_* 一致时可自动 Scan Context",
    )
    nav_delay_arg = DeclareLaunchArgument(
        "nav_stack_delay_sec",
        default_value="6.0",
        description="Gazebo spawn 后启导航栈（秒）",
    )
    reloc_after_rviz_arg = DeclareLaunchArgument(
        "reloc_after_rviz_sec",
        default_value="3.0",
        description="RViz 打开后再等多久开始 ScanContext+ICP（秒）",
    )
    reloc_delay_arg = DeclareLaunchArgument(
        "reloc_startup_delay_sec",
        default_value="10.0",
        description="导航栈启动后延时重定位（秒）",
    )
    initialpose_delay_arg = DeclareLaunchArgument(
        "initialpose_delay_sec",
        default_value="23.0",
        description="手动重定位时 /initialpose 延时（秒，自 launch 起）",
    )
    champ_delay_arg = DeclareLaunchArgument(
        "champ_control_delay_sec",
        default_value="12.0",
        description="四足 CHAMP 启动延时（秒）",
    )
    legged_settle_arg = DeclareLaunchArgument(
        "legged_settle_sec",
        default_value="25.0",
        description="四足站稳阶段零速时长（秒）",
    )
    ix_arg = DeclareLaunchArgument("initial_x", default_value="1.0")
    iy_arg = DeclareLaunchArgument("initial_y", default_value="-2.5")
    iz_arg = DeclareLaunchArgument(
        "initial_z",
        default_value="0.30",
        description="planar 0.30；legged 0.275",
    )
    ia_arg = DeclareLaunchArgument("initial_a", default_value="0.0")
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="legged",
        description="legged=Go2 四足真狗；planar=平面方块",
    )
    icp_planar_arg = DeclareLaunchArgument(
        "icp_planar_constraint",
        default_value="false",
        description="2_storey 楼梯建议 false",
    )
    legged_sim_arg = DeclareLaunchArgument(
        "legged_sim",
        default_value="true",
        description="四足仿真 TF（tf_manager_sim_legged）",
    )
    stair_nav_arg = DeclareLaunchArgument("stair_navigation", default_value="true")
    strict_path_arg = DeclareLaunchArgument("strict_global_path", default_value="true")
    champ_loco_arg = DeclareLaunchArgument(
        "champ_locomotion",
        default_value="false",
        description="仅 legged",
    )

    sim_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fl2_sim),
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
            "robot_model": LaunchConfiguration("robot_model"),
            "icp_planar_constraint": LaunchConfiguration("icp_planar_constraint"),
            "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
            "initialpose_delay_sec": LaunchConfiguration("initialpose_delay_sec"),
            "champ_control_delay_sec": LaunchConfiguration("champ_control_delay_sec"),
            "legged_settle_sec": LaunchConfiguration("legged_settle_sec"),
            "champ_locomotion": LaunchConfiguration("champ_locomotion"),
        }.items(),
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_stack),
        launch_arguments={
            "maps_root": LaunchConfiguration("maps_root"),
            "prior_pcd_path": LaunchConfiguration("prior_pcd_path"),
            "map_package_dir": LaunchConfiguration("map_package_dir"),
            "launch_rviz": "false",
            "launch_map_gui": LaunchConfiguration("launch_map_gui"),
            "launch_web": LaunchConfiguration("launch_web"),
            "launch_rosbridge": LaunchConfiguration("launch_rosbridge"),
            "web_http_port": LaunchConfiguration("web_http_port"),
            "reloc_auto_start": LaunchConfiguration("reloc_auto_start"),
            "reloc_startup_delay_sec": LaunchConfiguration("reloc_startup_delay_sec"),
            "scan_context_db": LaunchConfiguration("scan_context_db"),
            "launch_fastlio": "false",
            "use_sim_time": "true",
            "legged_sim": LaunchConfiguration("legged_sim"),
            "stair_navigation": LaunchConfiguration("stair_navigation"),
            "strict_global_path": LaunchConfiguration("strict_global_path"),
        }.items(),
    )

    delayed_nav = TimerAction(
        period=LaunchConfiguration("nav_stack_delay_sec"),
        actions=[nav_stack],
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter(name="use_sim_time", value=True))
    for a in (
        world_arg,
        prior_arg,
        map_pkg_arg,
        maps_root_arg,
        gui_arg,
        rviz_arg,
        rviz_delay_arg,
        map_gui_arg,
        web_arg,
        bridge_arg,
        web_port_arg,
        sc_db_arg,
        reloc_arg,
        nav_delay_arg,
        reloc_after_rviz_arg,
        reloc_delay_arg,
        initialpose_delay_arg,
        champ_delay_arg,
        legged_settle_arg,
        ix_arg,
        iy_arg,
        iz_arg,
        ia_arg,
        robot_model_arg,
        icp_planar_arg,
        legged_sim_arg,
        stair_nav_arg,
        strict_path_arg,
        champ_loco_arg,
    ):
        ld.add_action(a)
    ld.add_action(sim_loc)
    ld.add_action(delayed_nav)
    return ld
