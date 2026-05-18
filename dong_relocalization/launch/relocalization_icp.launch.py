from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("dong_relocalization")
    default_cfg = str(Path(pkg) / "config" / "relocalization_icp.yaml")

    cfg_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_cfg,
        description="YAML with icp_relocalizer ros__parameters",
    )

    node = Node(
        package="dong_relocalization",
        executable="icp_relocalizer_node",
        name="icp_relocalizer",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([cfg_arg, node])
