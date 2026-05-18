"""Mid-360 Livox 驱动节点（与官方 ``msg_MID360_launch.py`` 等价参数，但固定 ``user_config_path`` 为包内绝对路径）。

官方 launch 用 ``__file__`` 相对路径拼 ``../config/MID360_config.json``，在部分环境下易读到错误文件。
此处通过 ``get_package_share_directory`` 指向已安装 share 中的配置；改参数后可 ``livox_user_config_path:=`` 覆盖。

用法::

    ros2 launch dong_octomap mid360_livox_driver.launch.py
    ros2 launch dong_octomap mid360_livox_driver.launch.py livox_user_config_path:=/你的/MID360_config.json
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        livox_share = get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "未找到 livox_ros_driver2。请先在本工作区编译 Livox 驱动，或运行 scripts/sync_fastlio_mid360_deps.sh"
        ) from exc

    dong_octomap_share = get_package_share_directory("dong_octomap")
    go2_cfg = os.path.join(dong_octomap_share, "config", "MID360_go2_up_config.json")
    default_cfg = go2_cfg if os.path.isfile(go2_cfg) else os.path.join(
        livox_share, "config", "MID360_config.json"
    )
    if not os.path.isfile(default_cfg):
        raise RuntimeError(f"未找到 Mid-360 配置文件: {default_cfg}")

    cfg_arg = DeclareLaunchArgument(
        "livox_user_config_path",
        default_value=default_cfg,
        description="MID360 JSON 绝对路径（默认 dong_octomap Go2 顶装配置 MID360_go2_up_config.json）",
    )
    bd_arg = DeclareLaunchArgument(
        "livox_broadcast_code",
        default_value="livox0000000001",
        description="cmdline_input_bd_code；若无法发现雷达可改为设备广播码（见 Livox Viewer / 机身）",
    )

    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": LaunchConfiguration("livox_user_config_path"),
                "cmdline_input_bd_code": LaunchConfiguration("livox_broadcast_code"),
            }
        ],
    )

    return LaunchDescription([cfg_arg, bd_arg, driver])
