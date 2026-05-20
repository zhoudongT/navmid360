#!/usr/bin/env python3
"""为 Ericsii/FAST_LIO_ROS2（ros2 分支）的 CMakeLists.txt 打补丁（可重复执行）。

上游问题简述:
1. ``if($ENV{ROS_DISTRO} IN_LIST ...)`` 未加引号，且 ROS_DISTRO 为空时会误走 ``rosidl_get_typesupport_target``（旧发行版无此命令）。
2. colcon 子进程里 ROS_DISTRO 可能未设置。

补丁内容:
- 在 ``project(fast_lio)`` 后插入：从环境变量、CMAKE_PREFIX_PATH（/opt/ros/<distro>）解析 FASTLIO_ROS_DISTRO，否则默认 humble。
- 用带 ``elseif(COMMAND rosidl_get_typesupport_target)`` 的分支替换原 rosidl 链接块。

用法:
  python3 scripts/patch_fast_lio_ros2_cmake.py /path/to/fast_lio
"""
from __future__ import annotations

import pathlib
import re
import sys

MARKER_BEGIN = "# <<< dong_3d_nav: FAST_LIO ROS2 distro + rosidl patch BEGIN >>>\n"
MARKER_END = "# <<< dong_3d_nav: FAST_LIO ROS2 distro + rosidl patch END >>>\n"

INSERT_PROJECT_BLOCK = (
    MARKER_BEGIN
    + "# 解析 ROS 2 发行版：优先 ROS_DISTRO，其次 CMAKE_PREFIX_PATH 中的 /opt/ros/<distro>，默认 humble。\n"
    + "set(FASTLIO_ROS_DISTRO \"$ENV{ROS_DISTRO}\")\n"
    + "string(STRIP \"${FASTLIO_ROS_DISTRO}\" FASTLIO_ROS_DISTRO)\n"
    + "if(FASTLIO_ROS_DISTRO STREQUAL \"\")\n"
    + "  foreach(_fastlio_pfx ${CMAKE_PREFIX_PATH})\n"
    + "    if(_fastlio_pfx MATCHES \"/opt/ros/([a-zA-Z0-9_]+)$\")\n"
    + "      set(FASTLIO_ROS_DISTRO \"${CMAKE_MATCH_1}\")\n"
    + "      break()\n"
    + "    endif()\n"
    + "  endforeach()\n"
    + "endif()\n"
    + "if(FASTLIO_ROS_DISTRO STREQUAL \"\")\n"
    + "  set(FASTLIO_ROS_DISTRO \"humble\")\n"
    + "endif()\n"
    + "message(STATUS \"fast_lio: ROS_DISTRO resolved to '${FASTLIO_ROS_DISTRO}' (rosidl linkage)\")\n"
    + MARKER_END
)

ROSIDL_BLOCK = """list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if("${FASTLIO_ROS_DISTRO}" IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
elseif(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
else()
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()
"""


def main() -> int:
    if len(sys.argv) != 2:
        print("用法: python3 patch_fast_lio_ros2_cmake.py /path/to/fast_lio", file=sys.stderr)
        return 2
    root = pathlib.Path(sys.argv[1]).resolve()
    cm = root / "CMakeLists.txt"
    if not cm.is_file():
        print(f"未找到 {cm}", file=sys.stderr)
        return 1
    text = cm.read_text(encoding="utf-8")

    if MARKER_BEGIN in text and "elseif(COMMAND rosidl_get_typesupport_target)" in text:
        print(f"已是最新补丁，跳过: {cm}")
        return 0

    # 已打过 project 块则只更新 rosidl 块（允许重复运行）
    if MARKER_BEGIN not in text:
        m = re.search(r"(project\s*\(\s*fast_lio\s*\)\s*\n)", text)
        if not m:
            print("未找到 project(fast_lio)，拒绝修改。", file=sys.stderr)
            return 1
        text = text[: m.end()] + "\n" + INSERT_PROJECT_BLOCK + text[m.end() :]

    # 替换 list(APPEND EOL_LIST ... endif() 整段（紧挨 ament_target_dependencies）
    rosidl_pat = re.compile(
        r"list\s*\(\s*APPEND\s+EOL_LIST[^\n]*\)\s*"
        r"(?:\s*\n\s*if\s*\(\s*(?:\"\s*)?\$ENV\{ROS_DISTRO\}(?:\s*\")?\s+IN_LIST\s+EOL_LIST\s*\)\s*)"
        r"[\s\S]*?"
        r"endif\s*\(\s*\)\s*\n\s*(?=ament_target_dependencies\s*\()",
        re.MULTILINE,
    )
    new_text, n = rosidl_pat.subn(ROSIDL_BLOCK + "\n", text, count=1)
    if n != 1:
        print(
            "未能匹配并替换 rosidl 段落（上游 CMakeLists 结构是否已变？）。"
            f" 匹配次数={n}",
            file=sys.stderr,
        )
        return 1

    cm.write_text(new_text, encoding="utf-8")
    print(f"已更新: {cm}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
