from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("dong_dwa_local_planner")
    default_cfg = str(Path(pkg) / "config" / "dwa_local_planner.yaml")

    cfg_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_cfg,
        description="YAML with dwa_local_planner ros__parameters",
    )

    node = Node(
        package="dong_dwa_local_planner",
        executable="dwa_local_planner_node",
        name="dwa_local_planner",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([cfg_arg, node])
