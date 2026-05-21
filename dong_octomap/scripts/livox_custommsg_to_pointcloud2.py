#!/usr/bin/env python3
"""Livox CustomMsg -> sensor_msgs/PointCloud2，供 KISS-ICP 使用（ICP 仍用 CustomMsg）。"""

from __future__ import annotations

import struct

import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


def _make_xyz_time_cloud(msg: CustomMsg, frame_id: str) -> PointCloud2:
    n = int(msg.point_num)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="t",
            offset=12,
            datatype=PointField.FLOAT64,
            count=1,
        ),
    ]
    point_step = 20
    row_step = point_step * n
    buf = bytearray(row_step)
    base_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
    timebase = int(msg.timebase)
    for i, pt in enumerate(msg.points[:n]):
        off = i * point_step
        struct.pack_into("<fff", buf, off, pt.x, pt.y, pt.z)
        t_ns = base_ns + int(pt.offset_time) - timebase
        struct.pack_into("<d", buf, off + 12, float(t_ns) * 1e-9)
    out = PointCloud2()
    out.header = msg.header
    out.header.frame_id = frame_id
    out.height = 1
    out.width = n
    out.fields = fields
    out.is_bigendian = False
    out.point_step = point_step
    out.row_step = row_step
    out.is_dense = True
    out.data = bytes(buf)
    return out


class LivoxCustomMsgToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__("livox_custommsg_to_pointcloud2")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_pcd2")
        self.declare_parameter("output_frame_id", "livox_frame")
        inp = self.get_parameter("input_topic").get_parameter_value().string_value
        out = self.get_parameter("output_topic").get_parameter_value().string_value
        self._frame = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self._pub = self.create_publisher(PointCloud2, out, 10)
        self.create_subscription(CustomMsg, inp, self._cb, 10)
        self.get_logger().info(f"Livox {inp} -> PointCloud2 {out} (frame={self._frame})")

    def _cb(self, msg: CustomMsg) -> None:
        if msg.point_num == 0:
            return
        self._pub.publish(_make_xyz_time_cloud(msg, self._frame))


def main() -> None:
    rclpy.init()
    node = LivoxCustomMsgToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
