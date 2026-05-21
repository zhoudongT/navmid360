#!/usr/bin/env python3
"""收到 /icp_result 后重置 KISS-ICP 内部地图，与先验图重定位流程对齐。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty


class KissIcpResetOnIcp(Node):
    def __init__(self) -> None:
        super().__init__("kiss_icp_reset_on_icp")
        self.declare_parameter("icp_result_topic", "/icp_result")
        self.declare_parameter("reset_service", "kiss/reset")
        topic = self.get_parameter("icp_result_topic").get_parameter_value().string_value
        self._srv_name = self.get_parameter("reset_service").get_parameter_value().string_value
        self._client = self.create_client(Empty, self._srv_name)
        self.create_subscription(PoseWithCovarianceStamped, topic, self._on_icp, 10)
        self.get_logger().info(f"Listen {topic} -> reset {self._srv_name}")

    def _on_icp(self, _msg: PoseWithCovarianceStamped) -> None:
        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"服务 {self._srv_name} 不可用")
            return
        req = Empty.Request()
        future = self._client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info("KISS-ICP 已 reset（/icp_result）")
            if f.result() is not None
            else self.get_logger().warn("KISS-ICP reset 失败")
        )


def main() -> None:
    rclpy.init()
    node = KissIcpResetOnIcp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
