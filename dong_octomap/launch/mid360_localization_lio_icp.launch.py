"""Mid-360：Livox + FAST-LIO（定位向参数）+ ICP 全局重定位。

FAST-LIO（Ericsii ROS2）节点本身为 **LIO/增量建图**，不在主线内加载先验 PCD 做全局定位。
本 launch 的「localization」指工程化栈：

- **高频位姿 / 点云**：FAST-LIO（本配置降低地图发布、关闭 PCD 连续保存，便于与先验对齐配合）
- **先验地图全局对齐**：``dong_relocalization`` 的 ICP（``prior_pcd_path`` + ``/icp_relocalizer/trigger_icp``）

参数文件 ``mid360_fastlio_localization.yaml`` 位于 ``dong_octomap`` 包内，通过 ``mapping.launch.py`` 的
``config_path`` 指向本包 ``share/config``。

用法::

    ros2 launch dong_octomap mid360_localization_lio_icp.launch.py prior_pcd_path:=/path/to/map.pcd
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
            "未找到 livox_ros_driver2。请先编译 Livox 驱动或运行 scripts/sync_fastlio_mid360_deps.sh"
        ) from exc

    dong_octomap_share = get_package_share_directory("dong_octomap")
    livox_driver_launch = os.path.join(dong_octomap_share, "launch", "mid360_livox_driver.launch.py")
    if not os.path.isfile(livox_driver_launch):
        raise RuntimeError(f"未找到 {livox_driver_launch}")

    try:
        fast_lio_share = get_package_share_directory("fast_lio")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 fast_lio。请克隆 Ericsii/FAST_LIO_ROS2（ros2）并执行 scripts/patch_fast_lio_ros2_cmake.py 后 colcon build。"
        ) from exc

    get_package_share_directory("dong_relocalization")

    fastlio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    if not os.path.isfile(fastlio_launch):
        raise RuntimeError(f"未找到 {fastlio_launch}")

    reloc_share = get_package_share_directory("dong_relocalization")
    reloc_default_cfg = os.path.join(reloc_share, "config", "relocalization_icp.yaml")
    octomap_cfg_dir = os.path.join(dong_octomap_share, "config")

    livox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_driver_launch),
    )

    fastlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "rviz": "false",
            "config_path": octomap_cfg_dir,
            "config_file": "mid360_go2_up.yaml",
        }.items(),
    )

    prior_arg = DeclareLaunchArgument(
        "prior_pcd_path",
        default_value="",
        description="ICP 先验 PCD（map 系）；空则使用 relocalization_params_file 内配置",
    )
    params_file_arg = DeclareLaunchArgument(
        "relocalization_params_file",
        default_value=reloc_default_cfg,
        description="dong_relocalization ICP 参数文件",
    )

    ld = LaunchDescription()
    ld.add_action(prior_arg)
    ld.add_action(params_file_arg)
    ld.add_action(livox_include)
    ld.add_action(fastlio_include)
    ld.add_action(OpaqueFunction(function=_icp_node_with_optional_prior))
    return ld
