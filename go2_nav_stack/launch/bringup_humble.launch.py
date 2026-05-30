"""SingleMap：Livox + FAST-LIO2 + ICP + TF + 局部规划（EGO 或 FAR）。

用法::

    # 默认 EGO
    ros2 launch go2_nav_stack bringup_humble.launch.py

    # CMU FAR Planner（需先 build_far_planner_humble.sh）
    ros2 launch go2_nav_stack bringup_humble.launch.py local_planner:=far

局部规划：ego=EGO-Planner；far=CMU FAR；mppi=走廊+MPPI（推荐有全局路径时）。
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    go2_share = get_package_share_directory("go2_nav_stack")
    dong_share = get_package_share_directory("dong_octomap")
    cfg = os.path.join(go2_share, "config")
    frames_cfg = os.path.join(cfg, "frame_config.yaml")
    ego_launch = os.path.join(go2_share, "launch", "ego_planner_go2.launch.py")
    far_launch = os.path.join(go2_share, "launch", "far_planner_go2.launch.py")
    local_planner = LaunchConfiguration("local_planner")
    corridor_launch = os.path.join(go2_share, "launch", "corridor_mppi.launch.py")
    corridor_cfg = os.path.join(cfg, "corridor_mppi.yaml")

    maps_root = LaunchConfiguration("maps_root")
    prior_pcd = LaunchConfiguration("prior_pcd_path")
    launch_fastlio = LaunchConfiguration("launch_fastlio")
    launch_rviz = LaunchConfiguration("launch_rviz")
    reloc_auto_start = LaunchConfiguration("reloc_auto_start")
    reloc_startup_delay_sec = LaunchConfiguration("reloc_startup_delay_sec")

    fastlio_launch = os.path.join(
        dong_share, "launch", "mid360_fastlio2_localization.launch.py"
    )

    rviz_go2_ego_cfg = os.path.join(go2_share, "config", "rviz_go2_nav.rviz")
    rviz_go2_far_cfg = os.path.join(go2_share, "config", "rviz_go2_nav_far.rviz")
    fastlio_go2_cfg = os.path.join(dong_share, "config", "fastlio_go2_prior_map.yaml")
    icp_go2_cfg = os.path.join(dong_share, "config", "icp_go2_nav.yaml")

    fastlio2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        condition=IfCondition(launch_fastlio),
        launch_arguments={
            "prior_pcd_path": prior_pcd,
            "locate_in_prior_map": "true",
            "icp_planar_constraint": "true",
            "launch_rviz": launch_rviz,
            "rviz_config": LaunchConfiguration("rviz_config"),
            "publish_base_tf": "false",
            "send_odom_base_tf": "false",
            "enable_transform_publisher": "false",
            "odom_output_topic": "/lio_odom_raw",
            "prior_map_freeze_odom_without_features": "false",
            "prior_map_match_dist_sq": "121.0",
            "prior_map_min_effective_points": "8",
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_a": LaunchConfiguration("initial_a"),
            "icp_converged_count_thre": LaunchConfiguration("icp_converged_count_thre"),
            "fastlio_config_overlay": fastlio_go2_cfg,
            "icp_overlay_config": icp_go2_cfg,
            "imu_filter_enable": LaunchConfiguration("imu_filter_enable"),
            "imu_topic": LaunchConfiguration("fastlio_imu_topic"),
        }.items(),
    )

    seed_initialpose = Node(
        package="go2_nav_stack",
        executable="sim_initialpose_publisher",
        name="seed_initialpose_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("seed_initialpose")),
        parameters=[
            {
                "delay_sec": LaunchConfiguration("seed_initialpose_delay_sec"),
                "x": LaunchConfiguration("initial_x"),
                "y": LaunchConfiguration("initial_y"),
                "z": LaunchConfiguration("initial_z"),
                "yaw": LaunchConfiguration("initial_a"),
            }
        ],
    )

    stack = [
        Node(
            package="go2_nav_stack",
            executable="map_cloud_display_node",
            output="screen",
            parameters=[frames_cfg, os.path.join(cfg, "map_cloud_display.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="tf_manager_node",
            output="screen",
            parameters=[frames_cfg, os.path.join(cfg, "tf_manager.yaml")],
        ),
        Node(
            package="go2_nav_stack",
            executable="scan_context_manager",
            output="screen",
            parameters=[
                os.path.join(cfg, "relocalization.yaml"),
                {"maps_root": maps_root},
            ],
        ),
        TimerAction(
            period=0.5,
            actions=[
                Node(
                    package="go2_nav_stack",
                    executable="relocalization_manager",
                    output="screen",
                    parameters=[
                        os.path.join(cfg, "relocalization.yaml"),
                        {
                            "auto_reloc_on_start": ParameterValue(
                                reloc_auto_start, value_type=bool
                            ),
                            "accept_manual_initialpose": True,
                            "startup_delay_sec": ParameterValue(
                                reloc_startup_delay_sec, value_type=float
                            ),
                        },
                    ],
                ),
            ],
        ),
        Node(
            package="go2_nav_stack",
            executable="single_map_manager",
            output="screen",
            parameters=[
                os.path.join(cfg, "single_map_manager.yaml"),
                {"maps_root": maps_root},
            ],
        ),
        Node(
            package="go2_nav_stack",
            executable="global_path_manager",
            output="screen",
            parameters=[corridor_cfg],
            condition=IfCondition(
                PythonExpression(
                    ["'", local_planner, "' == 'mppi'"]
                )
            ),
        ),
        Node(
            package="go2_nav_stack",
            executable="global_path_server",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ego_launch),
            condition=IfCondition(
                PythonExpression(
                    ["'", local_planner, "' == 'ego'"]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(far_launch),
            condition=IfCondition(
                PythonExpression(
                    ["'", local_planner, "' == 'far'"]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(corridor_launch),
            condition=IfCondition(
                PythonExpression(
                    ["'", local_planner, "' == 'mppi'"]
                )
            ),
        ),
        Node(
            package="go2_nav_stack",
            executable="navigation_state_machine",
            output="screen",
        ),
    ]

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "local_planner",
            default_value="ego",
            description="局部规划器: ego | far | mppi（走廊+MPPI）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_go2_ego_cfg,
            description="RViz 配置；FAR 模式请用 rviz_go2_nav_far.rviz",
        )
    )
    ld.add_action(DeclareLaunchArgument("maps_root", default_value="/home/allen/maps"))
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_path",
            default_value="/home/allen/maps/scans_0.5m.pcd",
        )
    )
    ld.add_action(DeclareLaunchArgument("launch_fastlio", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "imu_filter_enable",
            default_value="true",
            description="对 /livox/imu 卡尔曼滤波，FAST-LIO 使用 /livox/imu_filtered",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "fastlio_imu_topic",
            default_value="/livox/imu_filtered",
            description="FAST-LIO/ICP IMU 话题；关闭滤波时请设为 /livox/imu",
        )
    )
    ld.add_action(DeclareLaunchArgument("launch_rviz", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "reloc_auto_start",
            default_value="true",
            description="true=开机 Scan Context+ICP；false=手动 /initialpose 或 seed_initialpose",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "reloc_startup_delay_sec",
            default_value="10.0",
            description="Livox+ICP 就绪后触发 Scan Context 的延时（秒）",
        )
    )
    ld.add_action(DeclareLaunchArgument("initial_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("initial_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("initial_z", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("initial_a", default_value="0.0"))
    ld.add_action(
        DeclareLaunchArgument(
            "icp_converged_count_thre",
            default_value="8",
            description="ICP 连续收敛帧数，越小锁定越快（默认 8）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "seed_initialpose",
            default_value="false",
            description="true=延时发布 /initialpose 种子（仅 ICP，不跑 Scan Context）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "seed_initialpose_delay_sec",
            default_value="7.0",
            description="seed_initialpose 延时（秒，应晚于 icp_node 3s 启动）",
        )
    )
    ld.add_action(fastlio2)
    ld.add_action(seed_initialpose)
    for n in stack:
        ld.add_action(n)
    return ld
