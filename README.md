# dong_3d_nav

[English](./README.en.md)

一套基于 ROS 2 Humble 的 3D 导航系统，通过 Web 界面交互。本系统面向宇树 Go2，并采用 Livox Mid-360 与 [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)（`ros2` 分支）提供 SLAM / 导航点云与里程计。

本目录包含五个 ROS 2 包：

- `dong_map_msgs`：地图包保存、加载、导出等自定义服务接口。
- `dong_octomap`：OctoMap 管理包，负责多种地图格式导入、地图包保存/加载、OctoMap 可视化和编辑。
- `octo_planner`：基于 OctoMap 的 3D 路径规划、路径跟踪控制和 Web 测试/导航 launch。
- `dong_dwa_local_planner`：可选的 DWA 局部速度规划（算法源自 MoveBase3D `local_planner_3d`，见包内说明）。
- `dong_relocalization`：自建 **ICP 点云重定位**（先验 PCD + 当前点云，Open3D），与 FAST-LIO 主线解耦，见下文。

## 功能概览

- 将 PCD 点云地图导入为 OctoMap。
- 将 ROS 2D 栅格地图导入为 3D OctoMap。
- 将 Gazebo `.world` / `.sdf` 场景转换为 OctoMap。
- 保存、加载 OctoMap 地图包。
- 使用 Qt/VTK GUI 查看和编辑 OctoMap 栅格。
- 使用 Web 页面查看 OctoMap、选择起点/终点并进行路径规划。
- 提供面向宇树 Go2、Livox Mid-360 + FAST-LIO 的导航入口和独立网页测试入口。

## 介绍视频

- Bilibili：[【开源】基于ROS2的3D导航系统](https://www.bilibili.com/video/BV1jgR9BmELw)
- YouTube：[【开源】基于ROS2的3D导航系统](https://www.youtube.com/watch?v=CepO90mzIeI)

## 目录结构

```text
dong_3d_nav/
├── dong_map_msgs/        # 自定义 srv 接口
├── dong_octomap/         # OctoMap 导入、管理、编辑、Web/GUI 工具
├── dong_dwa_local_planner/  # 可选 DWA 局部规划（源自 MoveBase3D）
├── dong_relocalization/     # ICP 重定位（先验 PCD + 实时点云）
├── octo_planner/        # 3D 路径规划、控制器、导航 launch
├── worlds/              # 示例 Gazebo world
└── install_deps_humble.sh
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- OctoMap / octomap_msgs
- OpenCV
- Open3D C++（`octomap_open3d_viewer_node`、`pcd_to_octomap_node`）：Ubuntu 22.04 无官方 `libopen3d-dev`。运行 `install_deps_humble.sh` 会安装 **LLVM `libc++-dev` / `libc++abi-dev`**，并把官方 **Open3D 0.17 x86_64 devel** 解压到 **`~/.local/opt/open3d-0.17.0-cxx11`**；编译前请设置  
  `export CMAKE_PREFIX_PATH="$HOME/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"`  
  （脚本结束时的 “Next steps” 里也有同样示例。）
- PyQt5、VTK、NumPy、Pillow、PyYAML；地图管理/编辑 GUI 还需 **`ros-humble-sensor-msgs-py`**（`sensor_msgs_py`，已写入 `install_deps_humble.sh`）
- 可选：`rosbridge_server`，用于 Web 页面通过 websocket 访问 ROS

运行整机导航 `octo_planner/launch/nav.launch.py` 前，请在工作区额外准备：

- [`go2_description`](https://github.com/Unitree-Go2-Robot/go2_description)（Go2 URDF；`nav.launch.py` 默认加载其中 `urdf/go2_description.urdf`）
- [`unitreerobotics/unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2)（Go2 控制与网络配置；[Unitree 开发者支持](https://support.unitree.com/home/en/developer)）
- [`Ericsii/FAST_LIO_ROS2`](https://github.com/Ericsii/FAST_LIO_ROS2)（`ros2` 分支；`fast_lio` + 默认 `config/mid360.yaml`）
- [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)（Mid-360 点云；通常作为 FAST-LIO 工作区依赖一并编译）

可将上述 **FAST_LIO_ROS2** 与 **livox_ros_driver2** 一键克隆到 **本仓库根目录**下（与 `octo_planner` 等并列，目录名为 `fast_lio`、`livox_ros_driver2`；已在 `.gitignore` 中忽略）：

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash scripts/sync_fastlio_mid360_deps.sh
```

若你曾把二者克隆在 `dong_3d_nav` 的上一级 `src` 里，可在本仓库根目录执行：`mv ../fast_lio ../livox_ros_driver2 .`（目标目录不存在同名文件夹时）。

### FAST_LIO_ROS2：ROS 2 发行版与「定位」

- **发行版 / CMake**：上游 `CMakeLists.txt` 用 `ROS_DISTRO` 选择 `rosidl_target_interfaces` 与 `rosidl_get_typesupport_target`；在 **未 `source` ROS** 或 **`ROS_DISTRO` 为空** 时易配置失败。克隆后请执行 **`bash scripts/apply_fast_lio_ros2_patches.sh`**（`sync_fastlio_mid360_deps.sh` 已自动调用），再 `colcon build --packages-select fast_lio`。补丁会写入 **`FASTLIO_ROS_DISTRO`**：依次读环境变量、`CMAKE_PREFIX_PATH` 中的 `/opt/ros/<distro>`，否则默认 **humble**。
- **定位能力说明**：Ericsii 主线 **`fastlio_mapping` 为 LIO / 增量建图**，`map_file_path` 在源码侧用于 **保存** 地图，**不是**加载先验 PCD 做全局定位。若要在先验地图上全局对齐，可使用本仓库 **`dong_relocalization`（ICP）** + FAST-LIO（见 `mid360_localization_lio_icp.launch.py`），或使用下节 **Fast-LIO2-Localization**；亦可参考 [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) 等其它 ROS 2 移植。
- **推荐一键（LIO 定位向参数 + ICP）**：`ros2 launch dong_octomap mid360_localization_lio_icp.launch.py prior_pcd_path:=/path/to/map.pcd`（FAST-LIO 使用 `dong_octomap/config/mid360_fastlio_localization.yaml`：降低地图发布、关连续 PCD 保存、默认可关外参在线估计；ICP 参数仍见 `dong_relocalization/config/relocalization_icp.yaml`）。

### Fast-LIO2-Localization（PolarisXQ，先验地图 + 内置 ICP）

若需要 **在修改版 FAST-LIO2 内加载先验地图并由 `icp_relocalization` 做初对齐**（与 Ericsii 主线行为不同），可克隆 **[PolarisXQ/Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization)**。

**若需同时把 Ericsii Fast-LIO2 与 Fast-LIO2-Localization 都下载到本工程根目录**（仅克隆、不解决 colcon 二选一）：

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash scripts/sync_fastlio2_and_localization.sh
```

仅拉取 Polaris 仓库时：

```bash
bash scripts/sync_fastlio2_localization.sh
```

- **与 Ericsii `fast_lio` 互斥**：该仓库内 ROS 包名仍为 **`fast_lio`**，与 `scripts/sync_fastlio_mid360_deps.sh` 拉取的 Ericsii **不能在同一 colcon 工作区同时编译**。请移走/排除其一后再 `colcon build`。
- **编译**：`colcon build --symlink-install --packages-select fast_lio icp_relocalization --paths .../Fast-LIO2-Localization/FAST_LIO .../Fast-LIO2-Localization/icp_relocalization`（脚本结束时会打印示例）。克隆后会对其中 **`FAST_LIO/CMakeLists.txt`** 自动应用与本仓库相同的 **ROS 发行版 + rosidl** 补丁。
- **运行**：先按上游 README 修改 **`example.launch.py`** 的 `map_path` 与 **`FAST_LIO/config/fast_lio_relocalization_param.yaml`** 的 `prior_map_path`（须一致）及初值；再启动  
  `ros2 launch dong_octomap mid360_fastlio2_localization.launch.py`  
  （会先起 **Livox**，再 include 上游 **example.launch.py**，内含 RViz 与延时启动的 ICP + FAST-LIO）。若仓库不在默认相对路径，可设环境变量 **`FASTLIO2_LOCALIZATION_ROOT`**。

### Livox Mid-360（livox_ros_driver2）常见问题

1. **切勿在 `livox_ros_driver2` 里执行官方 `./build.sh ROS2`**：该脚本会 **`cd ../../`** 再 `colcon build`，假定目录为 `工作区/src/livox_ros_driver2`。若 Livox 克隆在 **`dong_3d_nav/livox_ros_driver2`**，`../../` 会落到家目录等，colcon 会全盘扫到 Qt / conda / numpy 等无关 `setup.py`，终端里出现成串 **`python_setup_py` / `SyntaxError`**。请在工作区根目录使用 **`--paths` 只指向本仓库内的 livox 目录**，或运行本仓库 **`bash scripts/build_livox_ros_driver2.sh ~/你的/ros2_ws`**（需已 `source /opt/ros/humble/setup.bash`）。
2. **编译阶段 CMake 报错找不到 `liblivox_lidar_sdk_shared.so` / `livox_lidar_api.h`**：须先按 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) 官方说明编译并 **`sudo make install`**，使库与头文件出现在 **`/usr/local/lib`**、**/usr/local/include**（或按官方路径设置 `CMAKE_PREFIX_PATH`）。
3. **运行日志 `Init lds lidar fail!`**：多为 **`MID360_config.json`** 路径错误、JSON 语法错误，或 **Livox-SDK2 版本与驱动不匹配**。改配置后若未用 symlink 安装，需 **`colcon build` 再 `source install`**。推荐用本仓库 **`ros2 launch dong_octomap mid360_livox_driver.launch.py`**，默认使用包内 **绝对路径** 指向 `MID360_config.json`；也可用 `livox_user_config_path:=/你的路径/MID360_config.json`。
4. **网络**：`host_net_info` 里的本机 IP 必须是 **与雷达通信的那块网卡**上真实存在的地址（例如 `192.168.123.220`）；`lidar_configs` 里的 **`ip`** 必须是 **雷达当前 IP**（可用 Livox Viewer 或路由器里查看）。PC 与雷达需 **二层互通**，必要时临时关闭防火墙或放行 **UDP 56100–56501** 一带端口。
5. **无点云 / 收不到 IMU**：确认 **`xfer_format:=1`**（CustomMsg）与 FAST-LIO `mid360.yaml` 中 `lidar_type: 1` 一致；用 `ros2 topic echo /livox/lidar --no-arr` 看是否有数据。
6. **`cmdline_input_bd_code`**：官方 launch 默认占位串；若官方工具要求，可改为设备 **广播码**（`ros2 launch ... livox_broadcast_code:=你的码`）。

仅编译本仓库三个 ROS 2 包时，可不克隆上述仓库；整机导航需在同一工作区编译 **FAST_LIO_ROS2**（默认 `mid360.yaml`）及 **livox_ros_driver2**（Mid-360 驱动，随多数 FAST-LIO 分支依赖），并与 `unitree_ros2` 衔接 `/cmd_vel` 与 TF。`nav.launch.py` 已不再依赖原智元 `d1_bringup` / `d1_core`。

### Go2 实车推荐栈（KISS-ICP + Scan Context，pitch=18.5°）

工程方案见 **[FAST-LIO2_ScanContext_Quadruped_SingleMap_Humble.md](./FAST-LIO2_ScanContext_Quadruped_SingleMap_Humble.md)**（**单张**全局 PCD + OctoMap，不切 map、**无需楼梯/楼层配置**）。本仓库 **`go2_nav_stack`**：

| 模块 | 实现 |
|------|------|
| 驱动 + 里程计 | Livox + **KISS-ICP**（`mid360_kiss_icp_localization`） |
| 单地图 | `single_map_manager` + `maps/scans_0.5m.pcd` |
| 全局重定位 | **Scan Context** + **ICP** |
| TF | `tf_manager_node`（`map→odom` + `odom→base`） |
| 全局路径 | 网页规划 → `/planned_path` → `d1_controller` 跟点 |
| **实时避障** | `global_obstacle_avoidance_node`（0.3–1.0m 减速，0.1–0.3m 绕障） |
| 速度输出 | `d1_controller` → `/cmd_vel_raw` → 避障 → `/cmd_vel` |

**一键启动：**

```bash
source install/setup.bash
colcon build --packages-select go2_nav_stack kiss_icp icp_relocalization dong_octomap octo_planner

ros2 launch go2_nav_stack go2_full_stack.launch.py \
  map_package_dir:=/home/allen/maps/window_filtered_all \
  prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd
```

**两终端拆分：**

```bash
# 终端 A：bringup（文档 §26）
ros2 launch go2_nav_stack bringup_humble.launch.py prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd

# 终端 B：规划 + 网页
ros2 launch go2_nav_stack navigation_humble.launch.py map_package_dir:=/home/allen/maps/window_filtered_all
```

避障参数：`go2_nav_stack/config/global_obstacle_avoidance.yaml`

生成 Scan Context 库：

```bash
ros2 run go2_nav_stack build_scan_context_db --pcd /home/allen/maps/scans_0.5m.pcd --x 0 --y 0 --yaw 0
```

开机约 10s 后自动 Scan Context 查询；RViz **2D Pose Estimate** 仍可手动精调。Livox 默认配置：**`MID360_go2_tilt18_config.json`（pitch=18.5°）**。

## 安装依赖

可以使用仓库内脚本安装常用依赖：

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash install_deps_humble.sh
```

如果 CMake 找不到 Open3D，请确认已执行依赖脚本并设置了上文的 `CMAKE_PREFIX_PATH`；或自行安装带 `Open3DConfig.cmake` 的 Open3D C++ 开发树。

## 编译

若本机装有 **Miniconda / Anaconda**，请先 **`conda deactivate`** 直到命令行不再带 `(base)` 等环境名，再编译；否则 CMake 容易选到 conda 里的 Python（例如 3.13），与 Humble 自带的 ROSIDL / `empy` 不兼容，`dong_map_msgs` 会在生成接口时报错。依赖安装脚本会在运行时临时从 `PATH` 中去掉 conda 目录；你在自己终端里 **`colcon build`** 时仍需自行退出 conda 或显式指定系统解释器（见下）。

从 ROS 2 工作区根目录编译：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

若未把 Open3D 安装到 `~/.local/opt/open3d-0.17.0-cxx11`，请**不要**在环境里保留指向不存在目录的 `CMAKE_PREFIX_PATH`（可先 `unset CMAKE_PREFIX_PATH`），以免 colcon 出现 “path doesn't exist” 警告。

如果源码目录移动过，旧 CMake 缓存可能还指向旧路径，可以清理缓存后重编：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-clean-cache --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 地图导入

### 导入 PCD 点云地图

```bash
ros2 launch dong_octomap import_pcd_map.launch.py
```

`pcd_to_octomap_node` 使用包内 **最小 PCD 读取器**（ASCII / binary / binary_compressed + zlib），**不再依赖 Open3D C++**；Open3D C++ 仅用于 `octomap_open3d_viewer_node` 等可选节点。

该 launch 会启动：

- `pcd_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `pcd_map_import_gui`
- `octo_planner/dong_path_node`

### 导入 ROS 2D 栅格地图

```bash
ros2 launch dong_octomap import_ros_map.launch.py
```

该 launch 会启动：

- `occupancy_grid_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `ros_map_import_gui`
- `octo_planner/dong_path_node`

### 导入 Gazebo World / SDF

加载包内示例 world 时，推荐使用 `world_name`：

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=field.world
```

加载外部 world 文件时，继续使用绝对路径：

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_file:=/absolute/path/to/map.world
```

如果同时传入 `world_file` 和 `world_name`，优先使用 `world_file`。

`dong_octomap/worlds/` 目录内提供了两个示例 world 文件，并会随 `dong_octomap` 包安装到 `share/dong_octomap/worlds/`：

- `2_storey.world`：双层建筑/楼层示例。
- `field.world`：场地示例。

加载双层建筑示例：

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=2_storey.world
```

加载场地示例：

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=field.world
```

该 launch 会启动：

- `world_to_octomap_node`
- `world_selector_gui.py`
- `map_package_manager`
- `octo_planner/dong_path_node`

## 地图管理与编辑

OctoMap 管理和编辑主入口：

```bash
ros2 launch dong_octomap map_manager.launch.py
```

该 launch 会启动：

- `map_package_manager`
- `octomap_to_occupied_markers_node`
- `map_viewer_gui`
- 可选 `octo_planner/dong_path_node`

`map_viewer_gui` 支持：

- 打开地图包
- 刷新地图
- 保存地图
- 查看占据、禁行、可通行、风险代价图层
- 编辑栅格：`occupied`、`preblocked`、`traversable`、`clear`
- 选择起点、终点、导航目标

## Web 可视化

### ROS Humble：网页加载地图（推荐步骤）

1. **安装 rosbridge**（网页通过 WebSocket 连 ROS；未装则页面一直「未连接」）：

   ```bash
   sudo apt update
   sudo apt install ros-humble-rosbridge-server
   ```

2. **新开一个终端**，只加载 Humble 与工作区（避免与 Noetic / 其它 ROS 2 发行版混用）：

   ```bash
   source /opt/ros/humble/setup.bash
   cd /你的/colcon工作区   # 例如 ~/ros2_ws，且已 colcon build 过 dong_octomap 等包
   source install/setup.bash
   ```

3. **启动 Web + 规划 + 自动加载地图包**（把 `map_package` 换成你的地图包目录）：

   ```bash
   ros2 launch dong_octomap web_octomap.launch.py \
     map_package:=/home/allen/maps/window_filtered_all \
     launch_rosbridge:=true
   ```

   若约 2 秒后的自动 `load_package` 失败，可显式指定工作区 `setup.bash`（与编译安装位置一致）：

   ```bash
   ros2 launch dong_octomap web_octomap.launch.py \
     map_package:=/home/allen/maps/window_filtered_all \
     launch_rosbridge:=true \
     workspace_setup_bash:=/你的/colcon工作区/install/setup.bash
   ```

4. **浏览器**打开：`http://127.0.0.1:8080`（或 `http://本机IP:8080`）。若用手机访问，在页面里把 WebSocket 地址改成 `ws://跑ROS那台电脑的IP:9090`。

5. 页面点 **连接**；连接成功后，占据等图层会随话题更新。终端里应能看到 `LoadNavigationMapPackage` 成功、`octomap_to_occupied_markers` 发布占据体素数量等日志。

### 加载地图并启动 Web 页面

```bash
ros2 launch dong_octomap web_octomap.launch.py map_package:=~/maps/map launch_rosbridge:=true
```

常用参数：

- `map_package`：已保存的地图包目录。
- `http_port`：静态 Web 服务端口，默认 `8080`。
- `launch_rosbridge`：是否启动 `rosbridge_websocket`；**网页联调请设为 `true`**（并先安装 `ros-humble-rosbridge-server`）。
- `launch_map_gui`：是否同时启动 Qt 保存/加载窗口。
- `workspace_setup_bash`：自动 `load_package` 前额外 `source` 的路径；留空则尝试从本包安装路径推断工作区 `install/setup.bash`。
- **规划对齐**：`web_octomap.launch.py` 内 `dong_path_node` 与 `map_manager.launch.py` 使用相同参数。桌面 `map_viewer_gui` 发布 **VTK 拾取原始坐标**，由 `dong_path_node` 的 `findNearestFreeCell` 吸附。网页 **`main.js` 直接发布 `/start_point`、`/goal_point`**（不经过 `web_click_selector`）；射线命中可通行 mesh 后已将交点 **吸附到与 OctoMap 一致的体素中心** `(floor(x/r)+0.5)*r`，与 C++ `worldToGrid`/`gridToWorld` 一致，避免落在立方体表面任意点导致与桌面路径不一致。A* 邻居方向已排序。

浏览器访问：

```text
http://<机器人IP>:8080
```

### Web 功能测试

```bash
ros2 launch octo_planner web_test.launch.py
```

`web_test.launch.py` 用于测试网页访问、地图显示、Web 起终点选择、路径规划和基础控制链路。该 launch 不依赖 `go2_description`，会使用一个最小 `base_link` URDF 启动 `robot_state_publisher`。

启动前同样需要根据实际环境配置：

```text
octo_planner/config/nav_params.yaml
```

至少需要部署好：

- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

## 宇树 Go2 与 Livox Mid-360 完整导航

完整机器人导航入口：

```bash
ros2 launch octo_planner nav.launch.py
```

该 launch 面向宇树 Go2，并以 **Livox Mid-360 + FAST-LIO** 输出点云与里程计（`fast_lio` 的 `mapping.launch.py`，默认 `mid360.yaml`），同时包含 OctoMap 与 Web 规划链路。会启动或使用：

- `go2_description/urdf/go2_description.urdf`（由 `robot_state_publisher` 加载）
- `fast_lio`（FAST-LIO ROS2 建图/定位节点；需自行在工作区编译）
- `octo_planner/dong_path_node`
- `octo_planner/d1_controller`
- `dong_octomap/map_package_manager`
- Web viewer 和 `rosbridge_websocket`

运行前需要根据实际环境修改：

```text
octo_planner/config/nav_params.yaml
```

重点字段：

- `relocalization_bin_file`
- `map_package_dir`
- `relocalization_pcd_file`
- `show_rviz`
- `show_map_gui`
- `use_static_odom_to_base`

路径跟踪默认以 FAST-LIO 常用 **`body`** 系为机器人基座（可在 `nav_params.yaml` 的 `d1_controller` 中改 `robot_center_offset_frame`）。若 OctoMap 仍使用 `map` 系，请通过 `static_transform_publisher` 等方式发布 **`map` ↔ `camera_init`** 等与 FAST-LIO 文档一致的静态 TF，保证 RViz 与规划坐标一致。

`octo_planner/config/nav_params.yaml` 中至少需要部署好：

- `relocalization_bin_file`：重定位使用的 `.bin` 地图文件。
- `map_package_dir`：已经保存好的 OctoMap 地图包目录。

如果需要使用 RViz 观察定位效果，还需要部署：

- `relocalization_pcd_file`：用于 RViz 显示的 `.pcd` 点云地图文件。

### 规划层次说明（全局 / 局部 / 跟踪）

| 组件 | 角色 | 说明 |
|------|------|------|
| `dong_path_node` | **全局规划** | 在 OctoMap 体素网格上做 A*，发布 `/planned_path`。 |
| `d1_controller` | **路径跟踪（执行层）** | 订阅 `/planned_path`，经 `/start_navigation` 确认后按控制周期在机体坐标系下生成 `cmd_vel`，沿离散路点推进并做终点航向对准。**不是** Nav2 意义上的局部规划器：不使用局部代价地图，不做动态障碍下的在线重规划，也不实现 DWA/TEB 等局部轨迹优化。 |
| `dong_dwa_local_planner` | **可选 DWA 局部速度规划** | 从 [MoveBase3D](https://github.com/WilsonGuo/MoveBase3D) 的 `local_planner_3d` 提取的 DWA 采样与代价逻辑，独立为 `dong_dwa_local_planner` 包；订阅 `/planned_path` 与可选障碍物 `PointCloud2`，默认发布 `/cmd_vel_dwa`（默认 `enabled:=false`，与 `d1_controller` 并存时需自行选速或改话题）。见 `ros2 launch dong_dwa_local_planner dwa_local_planner.launch.py`。 |
| （Nav2 式局部规划） | **未内置** | 本仓库未集成 Nav2 `controller_server` / TEB 等；可选用上表 DWA 包或外接 Nav2。 |

`nav.launch.py` 与 `web_test.launch.py` 中可通过 `launch_controller:=true|false` 控制是否启动 `d1_controller`。

### 重定位（自建 ICP）

[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) 主线以建图/里程计为主；Mid-360 设备侧 `custom_map_mode: 2` + `.bin` 属于厂商链路。若要在 **ROS 图内** 自己做「先验地图 + 当前帧」对齐，可使用本仓库 **`dong_relocalization`**：

1. 依赖：`pip install open3d`（与 `pcd_file_publisher` 等脚本一致）。
2. 准备与 SLAM 输出 **同一坐标系** 的先验 PCD（通常为 `map` 下保存的全局图）。
3. 启动 FAST-LIO（或任意发布 `map`→`camera_init` TF 与注册点云 `/cloud_registered` 的源）。
4. 编辑 `dong_relocalization/config/relocalization_icp.yaml` 中的 `prior_pcd_path`（必填），按需改 `scan_topic`、`map_frame`、`lidar_frame`。
5. `ros2 launch dong_relocalization relocalization_icp.launch.py`，在机器人位姿初值大致正确时调用：  
   `ros2 service call /icp_relocalizer/trigger_icp std_srvs/srv/Trigger {}`  
   成功后在话题 **`/relocalization/lidar_pose_in_map`**（及 `..._cov`）得到 `camera_init`（或你配置的 `lidar_frame`）在 `map` 下的位姿；默认 **不** 抢发 TF（`publish_tf: false`），可与 FAST-LIO 并存，由你在 `static_transform_publisher` 或滤波器中融合。

**Mid-360 整机链路（驱动 + FAST-LIO + ICP）**：在已编译 `livox_ros_driver2`、`fast_lio`、`dong_relocalization` 后，可一条 launch 起传感器与里程计并挂 ICP 节点（须配置 Livox `MID360_config.json`，并准备先验 PCD）：

```bash
ros2 launch dong_octomap mid360_relocalization_icp.launch.py prior_pcd_path:=/绝对路径/你的地图.pcd
```

## 其他 Launch

```bash
ros2 launch dong_octomap octomap_test.launch.py
ros2 launch dong_octomap octomap_open3d.launch.py
ros2 launch dong_octomap mid360_slam.launch.py
ros2 launch dong_octomap mid360_loc.launch.py
ros2 launch dong_octomap mid360_livox_driver.launch.py
ros2 launch dong_octomap mid360_relocalization_icp.launch.py prior_pcd_path:=/path/to/map.pcd
ros2 launch dong_octomap mid360_localization_lio_icp.launch.py prior_pcd_path:=/path/to/map.pcd
ros2 launch dong_octomap mid360_fastlio2_localization.launch.py
```

`mid360_livox_driver.launch.py` 仅启动 Mid-360 驱动（推荐单独调通网络与话题）。`mid360_slam.launch.py` 与 `mid360_loc.launch.py` 会 **include** 工作区内的 `fast_lio/launch/mapping.launch.py`（默认 `mid360.yaml`），并附带 `dong_octomap` 的 RViz 与工具节点。`mid360_relocalization_icp.launch.py` 在此基础上再启动 **`mid360_livox_driver.launch.py`**（绝对路径读 `MID360_config.json`），并启动 `dong_relocalization` 的 ICP 节点（见上文「重定位」）。**`mid360_localization_lio_icp.launch.py`** 与前者类似，但 FAST-LIO 使用 **`mid360_fastlio_localization.yaml`**（更偏「与先验 ICP 联调」的 LIO 参数）。**`mid360_fastlio2_localization.launch.py`** 需已安装 [Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization)（与 Ericsii `fast_lio` 互斥），先起 Livox 再 include 上游 `example.launch.py`。

## 入门教材推荐

《机器人操作系统（ROS2）入门与实践》

<img src="./media/book_1.jpg" alt="机器人操作系统 ROS2 入门与实践" width="280">

淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)
