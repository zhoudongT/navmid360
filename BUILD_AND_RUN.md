# 编译与运行速查

本文档从主 [README.md](./README.md) 中抽出**依赖安装**、**编译**与**各运行入口**（shell 脚本与 `ros2 launch`），便于单独查阅。详细说明仍以 README 为准。

**环境**：Ubuntu 20.04、ROS 2 Foxy、`colcon`。整机导航还需额外工作区依赖（见 README「环境要求」）。

**本仓库路径**：`/home/allen/dong_3d_nav`（依赖安装脚本在此目录）。**ROS 2 工作区**：下文以 `/home/allen/ros2_ws` 为例；若你的工作区不在该路径，请把所有 `/home/allen/ros2_ws` 换成你的工作区根目录。

---

## 1. 依赖安装（Shell 脚本）

在本仓库根目录执行：

```bash
cd /home/allen/dong_3d_nav
bash install_deps_foxy.sh
```

脚本会处理 Foxy 相关依赖；若安装 Open3D C++ 开发包，通常解压到 `~/.local/opt/open3d-0.17.0-cxx11`。编译前需设置 `CMAKE_PREFIX_PATH`（见下一节）。

**FAST_LIO_ROS2 与 Livox Mid-360 驱动**（非 apt 依赖）：在本仓库根目录执行 `bash scripts/sync_fastlio_mid360_deps.sh`，会把 `fast_lio`、`livox_ros_driver2` 克隆到 **本仓库根目录内**（与 `octo_planner` 等并列，`.gitignore` 已排除）；随后在 `ros2_ws` 根目录 `colcon build` 时会与仓库内其它包一并被发现（详见 README）。

---

## 2. 编译（colcon）

### 2.1 注意事项

- 若使用 **Miniconda / Anaconda**，编译前请 **`conda deactivate`** 直到提示符无 `(base)` 等环境名；否则 CMake 可能选用 conda 的 Python，与 Foxy 的 ROSIDL / `empy` 不兼容。
- 若未把 Open3D 安装到默认路径，**不要**在环境中保留指向不存在目录的 `CMAKE_PREFIX_PATH`（可先 `unset CMAKE_PREFIX_PATH`）。

### 2.2 常规编译

从 **ROS 2 工作区根目录**执行（示例：`/home/allen/ros2_ws`，且 `src` 中已包含本仓库提供的五个包）：

```bash
cd /home/allen/ros2_ws
source /opt/ros/foxy/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner dong_dwa_local_planner dong_relocalization \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 2.3 清理 CMake 缓存后重编

源码目录移动过或路径异常时：

```bash
cd /home/allen/ros2_ws
source /opt/ros/foxy/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-clean-cache --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 2.4 `colcon` 提示 `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` 下某路径不存在

若出现类似 “The path `/home/allen/dong_3d_nav/install/...` … doesn't exist”，说明当前终端里仍带着**旧工作区**的叠加环境（例如曾在仓库根目录 `colcon build` 并 `source install/setup.bash`，或 `.bashrc` 里写错了 `source` 路径），而 **`/home/allen/dong_3d_nav/install` 并不存在或已被删掉**。`jie_octomap`、`jie_map_msgs` 等同理，来自历史/其它工作区的前缀。

**处理**：新开一个终端，或先清掉再只加载 Foxy 与工作区：

```bash
# 可选：先退出当前 shell 中所有 workspace 的叠加，再按顺序 source
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/foxy/setup.bash
cd /home/allen/ros2_ws   # 换成你的实际工作区根目录
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

并检查 `~/.bashrc` 是否错误地 `source /home/allen/dong_3d_nav/install/setup.bash`；若编译目录始终是 **`/home/allen/ros2_ws`**，应只 source 该工作区下的 `install/setup.bash`。

---

## 3. 运行入口（各脚本 / launch）

以下命令均假设已 `source /opt/ros/foxy/setup.bash` 且 `source /home/allen/ros2_ws/install/setup.bash`（或你的工作区 `install/setup.bash`）。

### 3.1 地图导入

| 说明 | 命令 |
|------|------|
| 导入 PCD 点云地图 | `ros2 launch dong_octomap import_pcd_map.launch.py` |
| 导入 ROS 2D 栅格地图 | `ros2 launch dong_octomap import_ros_map.launch.py` |
| 导入 Gazebo（包内示例 `field.world`） | `ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=field.world` |
| 导入 Gazebo（包内示例 `2_storey.world`） | `ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=2_storey.world` |
| 导入 Gazebo（外部 world，绝对路径） | `ros2 launch dong_octomap import_gazebo_world.launch.py world_file:=/absolute/path/to/map.world` |

说明：若同时传入 `world_file` 与 `world_name`，优先使用 `world_file`。

### 3.2 地图管理与 Web

| 说明 | 命令 |
|------|------|
| OctoMap 管理 / 编辑主入口 | `ros2 launch dong_octomap map_manager.launch.py` |
| 加载地图包并启动 Web | `ros2 launch dong_octomap web_octomap.launch.py map_package:=~/maps/map launch_rosbridge:=true` |

`web_octomap.launch.py` 常用参数：`map_package`、`http_port`（默认 `8080`）、`launch_rosbridge`（**网页联调需 `true`**，并安装 `ros-foxy-rosbridge-server`）、`launch_map_gui`、`workspace_setup_bash`（自动 `load_package` 失败时可指向工作区 `install/setup.bash`）。`dong_path_node` 参数与 `map_manager.launch.py` 一致；起终点由规划节点统一吸附，A* 邻居方向已排序。

### 3.3 规划与整机导航

| 说明 | 命令 |
|------|------|
| Web 功能测试（最小 URDF，不依赖 go2_description） | `ros2 launch octo_planner web_test.launch.py` |
| 宇树 Go2 + Livox Mid-360 + FAST-LIO 完整导航 | `ros2 launch octo_planner nav.launch.py` |

运行前请按 README 配置 `octo_planner/config/nav_params.yaml`（如 `relocalization_bin_file`、`map_package_dir` 等）。

### 3.4 其他 Launch（测试 / Open3D / Mid-360）

| 说明 | 命令 |
|------|------|
| OctoMap 测试 | `ros2 launch dong_octomap octomap_test.launch.py` |
| Open3D 相关 | `ros2 launch dong_octomap octomap_open3d.launch.py` |
| Mid-360 仅 Livox 驱动（绝对路径读配置，便于排障） | `ros2 launch dong_octomap mid360_livox_driver.launch.py` |
| Mid-360 定位（include 工作区内 `fast_lio`） | `ros2 launch dong_octomap mid360_loc.launch.py` |
| Mid-360 + FAST-LIO + ICP（先验 PCD；默认 `mid360.yaml`） | `ros2 launch dong_octomap mid360_relocalization_icp.launch.py prior_pcd_path:=/path/to/map.pcd` |
| Mid-360 + Fast-LIO2-Localization（先验图 + 上游 ICP；与 Ericsii `fast_lio` 互斥） | `ros2 launch dong_octomap mid360_fastlio2_localization.launch.py` |

---

## 4. 包内辅助脚本（非 launch，按需直接调用）

这些文件位于 `dong_octomap/scripts/`、`scripts/` 等目录，通常由 launch 或 GUI 间接调用；若需单独调试，请在 README 或对应 launch 中确认参数与依赖。其中 **`scripts/build_livox_ros_driver2.sh`** 用于在指定 ROS 2 工作区根目录下、用 `--paths` 仅编译本仓库内的 `livox_ros_driver2`（勿在 livox 目录执行官方 `./build.sh ROS2`）。**`scripts/apply_fast_lio_ros2_patches.sh`** 为上游 `fast_lio` 打 CMake（ROS 发行版 + rosidl）补丁。

| 路径（相对仓库根） |
|-------------------|
| `scripts/sync_fastlio2_and_localization.sh` |
| `scripts/sync_fastlio2_localization.sh` |
| `scripts/sync_fastlio_mid360_deps.sh` |
| `scripts/build_livox_ros_driver2.sh` |
| `scripts/patch_fast_lio_ros2_cmake.py` |
| `scripts/apply_fast_lio_ros2_patches.sh` |
| `dong_octomap/scripts/open3d_octomap_viewer.py` |
| `dong_octomap/scripts/pcd_file_publisher.py` |
| `dong_octomap/scripts/no_cache_http_server.py` |
| `dong_octomap/scripts/world_selector_gui.py` |
| `dong_octomap/scripts/save_bin_map_gui.py` |
| `dong_octomap/scripts/world_file_picker_gui.py` |
| `dong_octomap/scripts/web_click_selector.py` |
| `dong_octomap/scripts/test_map_package_save_smoke.py` |

Python 模块级入口（示例）：`dong_octomap/dong_octomap/` 下的 `pcd_map_import_gui.py`、`ros_map_import_gui.py`、`map_viewer_gui.py`、`map_package_manager.py`、`map_save_gui.py` 等，一般通过对应 launch 启动。

---

## 5. 浏览器访问 Web

在运行 `web_octomap.launch.py` 或整机 `nav.launch.py` 并开启静态服务后：

```text
http://<机器人或本机IP>:8080
```

端口以 launch 参数 `http_port` 为准。
