import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format = 1  # 0-Pointcloud2, 1-customized (CustomMsg)
multi_topic = 0
data_src = 0
publish_freq = 10.0
output_type = 0
frame_id = "livox_frame"
lvx_file_path = "/home/livox/livox_test.lvx"
cmdline_bd_code = "livox0000000001"
################### user configure parameters for ros2 end #####################


def _default_user_config_path() -> str:
    """默认与建图 launch 一致：dong_octomap MID360_go2_up_config.json（pitch=0）。"""
    try:
        dong_share = get_package_share_directory("dong_octomap")
        go2_cfg = os.path.join(dong_share, "config", "MID360_go2_up_config.json")
        if os.path.isfile(go2_cfg):
            return go2_cfg
    except PackageNotFoundError:
        pass
    cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
    return os.path.join(cur_path, "../config/MID360_config.json")


def generate_launch_description():
    cfg_arg = DeclareLaunchArgument(
        "user_config_path",
        default_value=_default_user_config_path(),
        description="Livox JSON；建图/定位请与 mid360_livox_driver 使用同一文件",
    )
    bd_arg = DeclareLaunchArgument(
        "cmdline_input_bd_code",
        default_value=cmdline_bd_code,
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": xfer_format},
            {"multi_topic": multi_topic},
            {"data_src": data_src},
            {"publish_freq": publish_freq},
            {"output_data_type": output_type},
            {"frame_id": frame_id},
            {"lvx_file_path": lvx_file_path},
            {"user_config_path": LaunchConfiguration("user_config_path")},
            {"cmdline_input_bd_code": LaunchConfiguration("cmdline_input_bd_code")},
        ],
    )

    return LaunchDescription([cfg_arg, bd_arg, livox_driver])
