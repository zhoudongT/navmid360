#!/usr/bin/env python3
"""对 Livox Mid-360 内置 IMU 做逐轴卡尔曼滤波与野值抑制，降低噪声对 FAST-LIO 里程计的干扰。

订阅原始 ``/livox/imu``，发布 ``/livox/imu_filtered``；FAST-LIO / ICP 应使用滤波后话题。
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class _ScalarKalman:
    """一维常值模型卡尔曼：抑制测量噪声，创新过大时视为野值仅预测。"""

    __slots__ = ("x", "p", "q", "r")

    def __init__(self, q: float, r: float, x0: float = 0.0) -> None:
        self.x = x0
        self.p = 1.0
        self.q = q
        self.r = r

    def predict(self) -> None:
        self.p += self.q

    def update(self, z: float, innov_gate: float) -> float:
        self.predict()
        innov = z - self.x
        if innov_gate > 0.0 and abs(innov) > innov_gate:
            return self.x
        s = self.p + self.r
        k = self.p / s if s > 1e-12 else 0.0
        self.x += k * innov
        self.p = max(1e-9, (1.0 - k) * self.p)
        return self.x


class LivoxImuKalmanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("livox_imu_kalman_filter")

        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/livox/imu_filtered")
        self.declare_parameter("gyro_process_noise", 1e-4)
        self.declare_parameter("gyro_measurement_noise", 2e-3)
        self.declare_parameter("accel_process_noise", 5e-4)
        self.declare_parameter("accel_measurement_noise", 0.05)
        self.declare_parameter("gyro_innovation_gate", 0.35)
        self.declare_parameter("accel_innovation_gate", 3.0)
        self.declare_parameter("max_gyro_rate_rad_s", 12.0)
        self.declare_parameter("max_accel_m_s2", 40.0)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("warmup_samples", 8)
        self.declare_parameter("gyro_bias_update_rate", 0.002)
        self.declare_parameter("stationary_gyro_norm", 0.08)
        self.declare_parameter("publish_diagnostic", False)

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        gq = self.get_parameter("gyro_process_noise").value
        gr = self.get_parameter("gyro_measurement_noise").value
        aq = self.get_parameter("accel_process_noise").value
        ar = self.get_parameter("accel_measurement_noise").value

        self._gyro_gate = float(self.get_parameter("gyro_innovation_gate").value)
        self._accel_gate = float(self.get_parameter("accel_innovation_gate").value)
        self._max_gyro = float(self.get_parameter("max_gyro_rate_rad_s").value)
        self._max_accel = float(self.get_parameter("max_accel_m_s2").value)
        self._median_n = max(1, int(self.get_parameter("median_window").value))
        self._warmup = max(0, int(self.get_parameter("warmup_samples").value))
        self._bias_alpha = float(self.get_parameter("gyro_bias_update_rate").value)
        self._stat_gyro = float(self.get_parameter("stationary_gyro_norm").value)
        self._diag = bool(self.get_parameter("publish_diagnostic").value)

        self._kf_g = [_ScalarKalman(gq, gr) for _ in range(3)]
        self._kf_a = [_ScalarKalman(aq, ar) for _ in range(3)]
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._gyro_hist: deque[list[float]] = deque(maxlen=self._median_n)
        self._acc_hist: deque[list[float]] = deque(maxlen=self._median_n)
        self._count = 0

        # 输入保持 sensor_data（通常 best_effort），输出改 reliable 兼容 FAST-LIO 默认订阅 QoS。
        out_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Imu, out_topic, out_qos)
        self.create_subscription(Imu, in_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"IMU KF: {in_topic} -> {out_topic}")

    @staticmethod
    def _median_axis(hist: deque[list[float]], idx: int) -> float:
        if not hist:
            return 0.0
        vals = sorted(row[idx] for row in hist)
        return vals[len(vals) // 2]

    def _clip_vec3(self, v: list[float], lim: float) -> list[float]:
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n <= lim or n < 1e-9:
            return v
        s = lim / n
        return [v[0] * s, v[1] * s, v[2] * s]

    def _cb(self, msg: Imu) -> None:
        g_raw = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        a_raw = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]

        g_raw = self._clip_vec3(g_raw, self._max_gyro)
        a_raw = self._clip_vec3(a_raw, self._max_accel)

        self._gyro_hist.append(g_raw)
        self._acc_hist.append(a_raw)

        if self._median_n > 1:
            g_in = [
                self._median_axis(self._gyro_hist, i) - self._gyro_bias[i] for i in range(3)
            ]
            a_in = [self._median_axis(self._acc_hist, i) for i in range(3)]
        else:
            g_in = [g_raw[i] - self._gyro_bias[i] for i in range(3)]
            a_in = a_raw

        g_norm = math.sqrt(g_in[0] ** 2 + g_in[1] ** 2 + g_in[2] ** 2)
        if g_norm < self._stat_gyro:
            for i in range(3):
                self._gyro_bias[i] = (1.0 - self._bias_alpha) * self._gyro_bias[
                    i
                ] + self._bias_alpha * g_raw[i]

        self._count += 1
        if self._count <= self._warmup:
            g_out = g_raw
            a_out = a_raw
            for i in range(3):
                self._kf_g[i].x = g_in[i]
                self._kf_a[i].x = a_in[i]
        else:
            g_out = [
                self._kf_g[i].update(g_in[i], self._gyro_gate) + self._gyro_bias[i]
                for i in range(3)
            ]
            a_out = [self._kf_a[i].update(a_in[i], self._accel_gate) for i in range(3)]

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = g_out[0]
        out.angular_velocity.y = g_out[1]
        out.angular_velocity.z = g_out[2]
        out.linear_acceleration.x = a_out[0]
        out.linear_acceleration.y = a_out[1]
        out.linear_acceleration.z = a_out[2]

        # 滤波后适当增大协方差对角，提示下游测量更可信
        g_var = max(1e-6, self.get_parameter("gyro_measurement_noise").value)
        a_var = max(1e-4, self.get_parameter("accel_measurement_noise").value)
        out.angular_velocity_covariance[0] = g_var
        out.angular_velocity_covariance[4] = g_var
        out.angular_velocity_covariance[8] = g_var
        out.linear_acceleration_covariance[0] = a_var
        out.linear_acceleration_covariance[4] = a_var
        out.linear_acceleration_covariance[8] = a_var

        self._pub.publish(out)

        if self._diag and self._count % 500 == 0:
            self.get_logger().info(f"IMU KF samples={self._count}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxImuKalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
