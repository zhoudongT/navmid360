"""Livox Mid-360 驱动 + FAST-LIO + 自建 ICP 重定位（dong_relocalization）一键链路。

依赖（须在同一工作区编译，可用本仓库 ``scripts/sync_fastlio_mid360_deps.sh`` 克隆到仓库根目录）:

- ``livox_ros_driver2``（由本包 ``mid360_livox_driver.launch.py`` 启动，CustomMsg ``xfer_format:=1``）
- ``fast_lio``（Ericsii/FAST_LIO_ROS2 ``ros2`` 分支，默认 ``mid360.yaml``：``/livox/lidar``、``/livox/imu``）
- ``dong_relocalization``（Open3D ICP；需 ``pip install open3d``）

先验地图 ``prior_pcd_path`` 须与 FAST-LIO 的 ``map`` 坐标系一致（通常为建图时保存的全局 PCD）。

用法示例::

    ros2 launch dong_octomap mid360_relocalization_icp.launch.py \\
      prior_pcd_path:=/path/to/map.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _icp_node_with_optional_prior(context, *args, **kwargs):
    """仅在命令行传入非空 prior_pcd_path 时覆盖 YAML，避免空字符串盖掉配置文件。"""
    params_file = LaunchConfiguration("relocalization_params_file")
    prior = LaunchConfiguration("prior_pcd_path").perform(context).strip()
    params = [params_file]
    if prior:
        params.append({"prior_pcd_path": prior})
    return [
        Node(
            package="dong_relocalization",
            executable="icp_relocalizer_node",
            name="icp_relocalizer",
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ROS 2 包 livox_ros_driver2。请克隆并编译 Livox 驱动，例如：\n"
            "  bash scripts/sync_fastlio_mid360_deps.sh\n"
            "  或: git clone https://github.com/Livox-SDK/livox_ros_driver2.git"
        ) from exc

    dong_octomap_share = get_package_share_directory("dong_octomap")
    livox_driver_launch = os.path.join(dong_octomap_share, "launch", "mid360_livox_driver.launch.py")
    if not os.path.isfile(livox_driver_launch):
        raise RuntimeError(f"未找到 {livox_driver_launch}")

    try:
        fast_lio_share = get_package_share_directory("fast_lio")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ROS 2 包 fast_lio。请克隆并编译 FAST_LIO_ROS2（ros2 分支），例如：\n"
            "  git clone -b ros2 https://github.com/Ericsii/FAST_LIO_ROS2.git fast_lio"
        ) from exc

    try:
        reloc_share = get_package_share_directory("dong_relocalization")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 ROS 2 包 dong_relocalization。请在本工作区编译本仓库中的 dong_relocalization 包。"
        ) from exc

    fastlio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    if not os.path.isfile(fastlio_launch):
        raise RuntimeError(f"未找到 {fastlio_launch}")

    reloc_default_cfg = os.path.join(reloc_share, "config", "relocalization_icp.yaml")

    livox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_driver_launch),
    )

    octomap_cfg_dir = os.path.join(dong_octomap_share, "config")

    fastlio_config_arg = DeclareLaunchArgument(
        "fastlio_config_file",
        default_value="mid360_go2_up.yaml",
        description="dong_octomap/config 下的 FAST-LIO YAML（Go2 顶装）",
    )

    fastlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "rviz": "false",
            "config_path": octomap_cfg_dir,
            "config_file": LaunchConfiguration("fastlio_config_file"),
        }.items(),
    )

    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="",
        description="先验地图 PCD 绝对路径（与 map 系一致）；空则沿用 params_file 内配置",
    )

    params_file_arg = DeclareLaunchArgument(
        "relocalization_params_file",
        default_value=reloc_default_cfg,
        description="dong_relocalization ICP 参数文件",
    )

    ld = LaunchDescription()
    ld.add_action(fastlio_config_arg)
    ld.add_action(prior_arg)
    ld.add_action(params_file_arg)
    ld.add_action(livox_include)
    ld.add_action(fastlio_include)
    ld.add_action(OpaqueFunction(function=_icp_node_with_optional_prior))
    return ld
