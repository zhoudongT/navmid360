#!/usr/bin/env python3
"""在 go2_nav2_ros2 的 go2_description 上生成 Mid-360 仿真 URDF 变体。

输出:
  xacro/accessories_mid360.urdf.xacro  — 仅 IMU（无 2D 激光）
  xacro/robot_mid360.xacro             — go2_robot + 狗头 Mid-360 + 仿真 ros2_control
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ACCESSORIES_MID360 = """<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- navmid360: IMU only (Mid-360 replaces 2D lidar) -->
  <xacro:macro name="accessories_mid360" params="base_name base_length base_height">
    <link name="imu_link">
      <inertial>
        <mass value="0.001"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="1e-09" ixy="0.0" ixz="0.0" iyy="1e-09" iyz="0.0" izz="1e-09"/>
      </inertial>
    </link>
    <joint name="imu_joint" type="fixed">
      <parent link="${base_name}"/>
      <child link="imu_link"/>
    </joint>
    <gazebo reference="imu_link">
      <sensor name="imu_controller" type="imu">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <namespace>/imu</namespace>
            <remapping>~/out:=data</remapping>
          </ros>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
"""

MID360_MOUNT_SNIPPET = """
  <!-- navmid360: 狗头 Mid-360；点云 pitch=19° 与 MID360_config.json 一致 -->
  <xacro:include filename="$(find go2_description)/xacro/mid360_head_mount.xacro"/>
  <xacro:go2_mid360_head_mount parent="trunk" topic="livox/lidar"/>
"""

MID360_HEAD_MOUNT = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find ros2_livox_simulation)/urdf/mid360.xacro"/>

  <xacro:property name="mid360_pitch_deg" value="19.0"/>
  <xacro:property name="mid360_pitch_rad" value="${mid360_pitch_deg * 3.1415926 / 180.0}"/>
  <xacro:property name="mid360_mesh_uri" value="file://$(find go2_description)/meshes/mid360.stl"/>
  <xacro:property name="mid360_head_x" value="0.168"/>
  <xacro:property name="mid360_head_y" value="0.0"/>
  <xacro:property name="mid360_head_z" value="0.087"/>

  <xacro:macro name="go2_mid360_head_mount" params="parent:=trunk topic:=livox/lidar">
    <link name="mid360_mount_bracket">
      <visual>
        <origin xyz="0.022 0 0.012" rpy="0 0 0"/>
        <geometry>
          <box size="0.050 0.052 0.010"/>
        </geometry>
        <material name="mid360_mount_gray">
          <color rgba="0.25 0.26 0.28 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0.040 0 0.030" rpy="0 ${mid360_pitch_rad} 0"/>
        <geometry>
          <box size="0.080 0.080 0.060"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.32"/>
        <origin xyz="0.040 0 0.030" rpy="0 0 0"/>
        <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
      </inertial>
    </link>
    <joint name="mid360_mount_bracket_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="mid360_mount_bracket"/>
      <origin xyz="${mid360_head_x} ${mid360_head_y} ${mid360_head_z}" rpy="0 0 0"/>
    </joint>
    <gazebo reference="mid360_mount_bracket">
      <material>
        <ambient>0.20 0.21 0.24 1</ambient>
        <diffuse>0.28 0.29 0.32 1</diffuse>
        <specular>0.08 0.08 0.08 1</specular>
      </material>
    </gazebo>
    <xacro:mid360
        name="livox_frame"
        parent="mid360_mount_bracket"
        topic="${topic}"
        extrinsic_roll="0"
        extrinsic_pitch="${mid360_pitch_deg}"
        extrinsic_yaw="0"
        visual_pitch_deg="${mid360_pitch_deg}"
        visual_mesh="${mid360_mesh_uri}"
        visual_mesh_scale="0.001 0.001 0.001">
      <origin xyz="0.048 0 0.030" rpy="0 0 0"/>
    </xacro:mid360>
  </xacro:macro>

</robot>
"""


def _ensure_mid360_assets(desc: Path) -> None:
    mount = desc / "xacro" / "mid360_head_mount.xacro"
    if not mount.is_file():
        mount.write_text(MID360_HEAD_MOUNT, encoding="utf-8")
    mesh = desc / "meshes" / "mid360.stl"
    if not mesh.is_file():
        print(f"[patch-mid360] 警告: 缺少 {mesh}，RViz/Gazebo 中雷达模型可能不可见", file=sys.stderr)


def _patch_robot_mid360(desc: Path) -> None:
    src = desc / "xacro" / "go2_robot.xacro"
    if not src.is_file():
        raise SystemExit(f"缺少 {src}，请先 sync go2_nav2_ros2")

    text = src.read_text(encoding="utf-8")
    text = text.replace('name="go2"', 'name="go2_mid360"', 1)
    text = text.replace(
        "$(find go2_description)/xacro/accessories.urdf.xacro",
        "$(find go2_description)/xacro/accessories_mid360.urdf.xacro",
    )
    text = text.replace("<xacro:accessories", "<xacro:accessories_mid360")
    text = re.sub(
        r"<parameters>\$\(find go2_description\)/config/[^<]+</parameters>",
        "<parameters>$(find go2_description)/config/ros_control/ros_control_sim.yaml</parameters>",
        text,
    )
    if "mid360_head_mount.xacro" not in text:
        text = text.replace("</robot>", MID360_MOUNT_SNIPPET + "\n</robot>")

    out = desc / "xacro" / "robot_mid360.xacro"
    out.write_text(text, encoding="utf-8")


def main() -> int:
    desc = Path(sys.argv[1] if len(sys.argv) > 1 else "go2_description").resolve()
    if not desc.is_dir():
        print(f"错误: 目录不存在 {desc}", file=sys.stderr)
        return 1

    (desc / "xacro").mkdir(parents=True, exist_ok=True)
    (desc / "config" / "ros_control").mkdir(parents=True, exist_ok=True)

    _ensure_mid360_assets(desc)
    (desc / "xacro" / "accessories_mid360.urdf.xacro").write_text(
        ACCESSORIES_MID360, encoding="utf-8"
    )
    _patch_robot_mid360(desc)

    print(f"[patch-mid360] OK: {desc / 'xacro' / 'robot_mid360.xacro'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
