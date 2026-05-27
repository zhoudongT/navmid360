#!/usr/bin/env python3
"""缓慢插值到站立角，并持续保持（直到收到门控关闭或 locomotion 接管）。"""

from __future__ import annotations

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

STAND_JOINTS = (
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
)
STAND_POSITIONS = (
    0.1,
    0.8,
    -1.5,
    -0.1,
    0.8,
    -1.5,
    0.1,
    1.0,
    -1.5,
    -0.1,
    1.0,
    -1.5,
)


class SimGo2SetStandPose(Node):
    def __init__(self) -> None:
        super().__init__("sim_go2_set_stand_pose")
        self.declare_parameter(
            "trajectory_topic",
            "/joint_group_effort_controller/joint_trajectory",
        )
        self.declare_parameter("gate_topic", "/sim_go2_stand_hold_enable")
        self.declare_parameter("ramp_sec", 3.0)
        self.declare_parameter("hold_hz", 50.0)
        self.declare_parameter("wait_controller_sec", 90.0)
        self.declare_parameter("hold_forever", True)

        topic = str(self.get_parameter("trajectory_topic").value)
        gate_topic = str(self.get_parameter("gate_topic").value)
        ramp_sec = max(0.5, float(self.get_parameter("ramp_sec").value))
        hold_hz = max(5.0, float(self.get_parameter("hold_hz").value))
        wait_sec = float(self.get_parameter("wait_controller_sec").value)
        self._hold_forever = bool(self.get_parameter("hold_forever").value)
        self._hold_enabled = True

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self.create_subscription(Bool, gate_topic, self._on_gate, 10)

        time.sleep(1.0)
        deadline = time.monotonic() + wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self._ramp(ramp_sec, hold_hz)
        if self._hold_forever:
            self.get_logger().info(
                f"进入持续站立保持（{hold_hz:.0f} Hz），门控话题 {gate_topic}"
            )
            period = 1.0 / hold_hz
            while rclpy.ok():
                if self._hold_enabled:
                    self._publish_stand(duration_sec=0.4)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
        else:
            self.get_logger().info("站立插值完成（未启用 hold_forever）")

    def _on_gate(self, msg: Bool) -> None:
        self._hold_enabled = bool(msg.data)

    def _make_stand_msg(self, alpha: float, duration_sec: float) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = list(STAND_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [alpha * p for p in STAND_POSITIONS]
        pt.velocities = [0.0] * len(STAND_JOINTS)
        pt.time_from_start = Duration(
            seconds=int(duration_sec), nanoseconds=int((duration_sec % 1.0) * 1e9)
        ).to_msg()
        msg.points = [pt]
        return msg

    def _publish_stand(self, duration_sec: float) -> None:
        self._pub.publish(self._make_stand_msg(1.0, duration_sec))

    def _ramp(self, ramp_sec: float, hz: float) -> None:
        n_steps = max(2, int(ramp_sec * hz))
        period = 1.0 / hz
        self.get_logger().info(f"缓慢站立：{ramp_sec:.1f}s 插值")
        for step in range(1, n_steps + 1):
            if not rclpy.ok():
                break
            alpha = step / n_steps
            self._pub.publish(self._make_stand_msg(alpha, ramp_sec * alpha))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def main() -> None:
    rclpy.init()
    node = SimGo2SetStandPose()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
