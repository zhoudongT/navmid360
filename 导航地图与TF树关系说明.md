# 导航中各个地图和 TF 树之间的关系

## 1. 核心概念

在机器人导航系统中，要先分清楚两个概念：

- **地图 Map**：表示环境数据，例如点云地图、二维栅格地图、高程地图、局部障碍物地图。
- **TF 树 Transform Tree**：表示各个坐标系之间的位置和姿态关系。

简单来说：

> 地图是“环境在哪里”，TF 是“机器人和传感器在环境里的哪里”。

导航系统是否稳定，很大程度取决于 TF 树是否清晰、唯一、连续。

---

## 0. 本仓库实机对照（`dong_3d_nav` / Go2 + Mid-360）

> 下文第 2–20 节为通用 Nav2/SLAM 说明；**以本节为准** 配置与排障一键栈（`./scripts/start_go2_nav.sh`）。

### 0.1 实际 TF 树（帧名以代码为准）

```text
map
 └── odom                    ← tf_manager（静态，重定位后锁定）
      └── base_link          ← tf_manager（/state_estimation，50Hz）
           └── livox_frame   ← tf_manager 静态（非 mid360_link）
                └── imu_link ← fastlio2 launch 静态 livox_frame→imu_link
```

| TF | 唯一发布者 | 说明 |
|---|---|---|
| `map → odom` | `tf_manager_node` | 订阅 `/relocalization/pose`；`T_map_odom = T_map_icp @ inv(T_odom_livox)` |
| `odom → base_link` | `tf_manager_node` | 订阅 `/state_estimation`（`planar_odom_filter` ← `/lio_odom_raw`） |
| `base_link → livox_frame` | `tf_manager_node` | 默认 z=0.15m；**不要**再发 `base→imu`（防双父节点） |
| `livox_frame → imu_link` | `fastlio2_localization.launch.py` | FAST-LIO `sensor_frame_id: imu_link` |

**禁止重复发布**（已在一键 launch 中关闭）：

- FAST-LIO：`send_odom_base_tf:=false`，话题 `/lio_odom_raw`
- ICP：`publish_map_to_odom_tf:=false`，只发 `/icp_result` 与 `/transformed_cloud`
- 无 KISS-ICP 参与一键导航（可选包保留，默认不用）

**map 与 odom 不要叠在一起**（`tf_manager.yaml`）：

- `seed_map_to_odom_identity: false` — 重定位**前**不发布 `map→odom`
- 仅 ICP 对**先验 map** 校准成功后，由 `tf_manager` 发布一次 `map→odom`（`latch_map_to_odom: true`）
- `odom` 仍由 FAST-LIO 连续更新（`odom→base_link`），重定位只“修正 map 与 odom 的偏差”，不改动 odom 原点逻辑

### 0.2 定位与重定位数据流

```text
Livox /livox/lidar (CustomMsg)
    → icp_node（先验 PCD 匹配）→ /icp_result、/transformed_cloud、/prior_map
    → Scan Context（可选）→ /initialpose
    → Qt「当前姿态」/ /initialpose
    → relocalization_manager → /relocalization/pose
    → tf_manager → map→odom（锁定）

FAST-LIO2 → /lio_odom_raw → planar_odom_filter → /state_estimation → tf_manager → odom→base_link
FAST-LIO2 → /cloud_registered（frame_id=odom，含 intensity）
    → map_cloud_display_node → /cloud_registered_map（frame_id=map，保留 intensity）
```

重定位锁定后 `/transformed_cloud` 使用 **实时 TF `map←livox_frame`**，勿用冻结 ICP 矩阵（否则会随机身旋转）。

### 0.3 地图 / 话题 / 坐标系

| 数据 | 话题 | frame_id | 用途 |
|---|---|---|---|
| 先验 PCD | `/prior_map` | `map` | ICP 目标图、RViz 白点云 |
| ICP 当前扫 | `/transformed_cloud` | `map` | 对齐检查（无 intensity） |
| LIO 实时扫 | `/cloud_registered` | `odom` | 局部避障输入 |
| 彩色显示 | `/cloud_registered_map` | `map` | RViz Intensity 着色 |
| OctoMap 包 | `/octomap` 等 | `map` | 网页/global_path |
| 全局路径 | `/planned_path` | `map` | dong_path_node |
| 局部 costmap | `/local_costmap/costmap` | `map` | Nav2 MPPI（见 0.4） |
| 里程计 | `/state_estimation` | `odom` | TF + 控制器 |

### 0.4 与通用文档的差异（本仓库刻意配置）

| 项目 | 通用 Nav2 建议 | 本仓库实际 | 原因 |
|---|---|---|---|
| `local_costmap.global_frame` | `odom` | **`map`** | 与 `/planned_path`、`/octomap` 同系，避免路径变换失败 |
| 雷达 frame 名 | `mid360_link` | **`livox_frame`** | `livox_ros_driver2` 驱动约定 |
| 主定位栈 | AMCL / KISS 等 | **FAST-LIO2 + ICP + Scan Context** | `go2_full_stack.launch.py` |
| 网页 HTTP | 8080/8081 | **8888**（`WEB_PORT`） | 静态页 `dong_octomap/web` |
| rosbridge WS | 9090 | **9090**（`ROSBRIDGE_PORT`） | 与 `main.js` 默认 `ws://主机:9090` 一致 |

### 0.5 启动与排障

```bash
cd ~/dong_3d_nav
source install/setup.bash
./scripts/stop_go2_nav.sh          # 释放 8888(网页)/9090(ws) 等端口
./scripts/start_go2_nav.sh
# 网页 http://127.0.0.1:8888 ；rosbridge ws://主机:9090
# 若启动后网页打不开：勿在 preflight 中误杀 oneclick（已修）；见 scripts/go2_nav_kill.sh
```

| 现象 | 处理 |
|---|---|
| `rosbridge [Errno 98] Address already in use` | `./scripts/stop_go2_nav.sh` 或 `fuser -k 9090/tcp` |
| `/transformed_cloud` 随机身转 | 重编 `icp_relocalization`；确认 `map→odom` 与 `odom→livox_frame` TF 连通 |
| `/cloud_registered_map` 无彩色 | RViz Color=**Intensity**；确认已 `RELOCALIZED` |
| 局部栅格 seemingly 随机器人转 | RViz **Fixed Frame=map**；勿用 `odom` 看 map 系 costmap |
| **局部地图不显示** | RViz **Map**→`/costmap/costmap`（QoS **Volatile**）；约 40s 后 `ros2 lifecycle get /costmap/costmap` 为 active |
| **局部图跟机器人** | 独立 costmap：`global_frame: base_link`（栅格绑机体，RViz Fixed Frame=map 下随狗走）；**须** `odom→base_link` TF 在动 |
| **转圈后 odom 飞走** | 已：FAST-LIO `extrinsic_est_en=false`、跳变剔除（planar_odom_filter）、重定位后须≥12 有效点才信 LIO；勿开 `prior_map_freeze_odom_without_features` |
| **rosbridge Errno 98** | 先 `./scripts/stop_go2_nav.sh`；栈内 rosbridge **延迟 12s** 由 `scripts/run_rosbridge_safe.sh` 单实例启动 |
| **Waiting for initial pose** | Qt「当前姿态」发 `/initialpose`；ICP fitness 须 **< 0.08**（日志 fitness≈0.15 表示初值偏差大） |
| **controller_server lifecycle 不可用** | 启动后约 **30–60s** 自动重试；勿在旧栈未停时再开第二个 `start_go2_nav.sh` |

配置文件：

- `go2_nav_stack/config/frame_config.yaml` — 统一帧名与话题
- `go2_nav_stack/config/tf_manager.yaml` — TF 发布职责
- `dong_octomap/config/icp_fastlio_localization.yaml`
- `go2_nav_stack/config/nav2_mppi_params.yaml`

自检：`bash scripts/verify_nav_tf.sh`（栈运行且已重定位后）。

---

## 2. 最标准的 TF 树结构

常见 ROS / ROS2 / Nav2 / SLAM 系统中，推荐使用下面这种结构：

```text
map
 └── odom
      └── base_link
           ├── imu_link
           ├── lidar_link
           ├── camera_link
           └── other_sensor_link
```

其中最关键的是：

```text
map -> odom -> base_link
```

不要让多个模块同时发布同一个 TF，否则很容易出现定位跳变、局部规划异常、导航失败。

---

## 3. 各坐标系含义

| 坐标系 | 含义 | 特点 | 通常由谁发布 |
|---|---|---|---|
| `map` | 全局地图坐标系 | 长期稳定，不随机器人短时间运动漂移 | SLAM / 定位模块 |
| `odom` | 里程计坐标系 | 短时间连续、平滑，但长期可能漂移 | 里程计 / LIO / VIO / robot_localization |
| `base_link` | 机器人本体中心坐标系 | 表示机器人机体中心 | 机器人状态估计 |
| `imu_link` | IMU 坐标系 | IMU 安装位置 | static TF / URDF |
| `lidar_link` | 激光雷达坐标系 | LiDAR 安装位置 | static TF / URDF |
| `camera_link` | 相机坐标系 | RGBD / 深度相机安装位置 | static TF / URDF |

---

## 4. map、odom、base_link 的关系

### 4.1 `map -> odom`

`map -> odom` 表示：

> 当前 odom 坐标系在全局 map 坐标系中的位置。

它通常由全局定位模块发布，例如：

- AMCL
- SLAM Toolbox
- FAST-LIO2 定位模式
- KISS-ICP + 先验地图
- Scan Context 重定位模块
- 自研全局定位模块

这个 TF **允许跳变**。

例如机器人重定位成功后，系统发现机器人实际位置和累计里程计位置不同，就可以通过修正 `map -> odom` 来校正全局位置。

---

### 4.2 `odom -> base_link`

`odom -> base_link` 表示：

> 机器人本体在 odom 坐标系下的实时连续运动。

它要求：

- 连续
- 平滑
- 低延迟
- 不能突然跳变

通常由以下模块发布：

- FAST-LIO2
- KISS-ICP odometry
- VIO
- leg odometry
- IMU + 轮速计融合
- `robot_localization`

局部规划器和控制器非常依赖这个 TF。

---

### 4.3 `base_link -> sensor_link`

例如：

```text
base_link -> mid360_link
base_link -> imu_link
base_link -> camera_link
```

这些一般是固定安装关系，应该由以下方式发布：

- URDF
- `robot_state_publisher`
- `static_transform_publisher`

这些 TF 一般不应该动态变化，除非传感器安装在云台、机械臂、升降机构上。

---

## 5. 为什么不能直接用 `map -> base_link`

有些系统会直接发布：

```text
map -> base_link
```

这种方式在简单显示中可以用，但在工程导航中不推荐。

原因是：

1. 全局定位可能会跳变。
2. 局部控制器需要连续平滑的位姿。
3. 如果 `base_link` 直接在 `map` 下跳动，局部规划器会受到影响。
4. Nav2 推荐使用 `map -> odom -> base_link` 的结构。

正确做法：

```text
map
 └── odom
      └── base_link
```

---

## 6. 各类地图之间的关系

导航系统中常见的地图不止一类。

```text
全局点云地图 / PCD Map
        ↓
二维栅格地图 / Occupancy Grid
        ↓
全局 Costmap
        ↓
局部 Costmap
        ↓
局部轨迹 / 控制指令
```

### 6.1 全局点云地图

常见格式：

- `.pcd`
- `.ply`
- `.las`
- OctoMap
- Voxel Map

作用：

- 全局定位
- 点云匹配
- 重定位
- 三维环境表达

通常坐标系：

```text
frame_id = map
```

---

### 6.2 二维栅格地图

常见格式：

- `.pgm + .yaml`
- OccupancyGrid

作用：

- Nav2 全局路径规划
- 静态障碍物表达
- 平面导航

通常坐标系：

```text
frame_id = map
```

---

### 6.3 高程地图 Elevation Map

四足机器人常用。

作用：

- 判断地面高度
- 判断台阶
- 判断斜坡
- 判断可通行区域
- 给落脚点规划提供参考

常见坐标系：

```text
frame_id = map
```

或者局部高程地图使用：

```text
frame_id = odom
```

---

### 6.4 全局 Costmap

作用：

- 给全局规划器使用
- 基于静态地图、障碍物、膨胀层生成代价地图

推荐配置：

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
```

说明：

- 全局代价地图应当放在 `map` 坐标系下。
- 这样全局规划器知道机器人在整张地图里的位置。

---

### 6.5 局部 Costmap

作用：

- 局部避障
- 动态障碍物处理
- 给控制器提供短距离环境信息

**通用 Nav2 推荐**（其他项目）：

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
```

**本仓库 `go2_nav_stack` 实际**（`config/nav2_mppi_params.yaml`）：

```yaml
local_costmap:
  global_frame: map
  robot_base_frame: base_link
  rolling_window: true
  # 障碍层点云：/cloud_registered（frame_id=odom），sensor_frame: odom
```

说明：

- 通用做法用 `odom` 是为避免重定位时局部图跳动。
- 本仓库全局路径、`dong_path`、OctoMap 均在 **`map`**，局部 costmap 也用 **`map`**，避免 MPPI 路径变换失败。
- RViz 显示局部图：**Fixed Frame=`map`**，话题 **`/costmap/costmap`**，`frame_id=base_link`（3m 窗口绑在机体上）。

---

## 7. Nav2 中的典型配置

> **本仓库**局部 costmap 使用 `global_frame: map`，见第 0.4 节；下列为通用模板。

Nav2 通常这样配置：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      transform_tolerance: 0.3

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 10.0
      publish_frequency: 5.0
      transform_tolerance: 0.3
```

核心逻辑：

```text
global_costmap 用 map
local_costmap 用 odom
controller 用 base_link
```

---

## 8. 本仓库定位栈（FAST-LIO2 + ICP + Scan Context）

> 一键导航 **不使用 KISS-ICP**；以下为 `go2_nav_stack` + `dong_octomap` 实机分工。

```text
先验 PCD（/prior_map, map）
        │
        ▼
icp_node ← /livox/lidar          Scan Context → /initialpose（可选）
        │                                    │
        └──────── /icp_result ───────────────┘
                      │
                      ▼
         relocalization_manager → /relocalization/pose
                      │
                      ▼
              tf_manager → map→odom（锁定）
                      │
map ◄─────────────────┴────────────────── odom ◄── FAST-LIO2 /lio_odom_raw
                                              │
                                         base_link
                                              ├── livox_frame（驱动+静态外参）
                                              └── imu_link
```

| 模块 | 负责内容 | 发布 TF / 话题 |
|---|---|---|
| `livox_ros_driver2` | 原始雷达 | `frame_id=livox_frame`，`/livox/lidar` |
| `fast_lio` | LIO 里程计与配准点云 | `/lio_odom_raw`、`/cloud_registered`（**不发** odom→base） |
| `planar_odom_filter` | 平面 odom | `/state_estimation` |
| `icp_node` | 先验图 ICP | `/icp_result`、`/transformed_cloud`、`/prior_map` |
| `scan_context_manager` | 粗定位候选 | `/scan_context/last_query` |
| `relocalization_manager` | 流程与 `/relocalization/pose` | 不发 TF |
| `tf_manager_node` | **唯一** map→odom、odom→base、静态外参 | `/tf`、`/tf_static` |
| `map_cloud_display_node` | 彩色点云到 map | `/cloud_registered_map` |

---

## 9. 本仓库四足 + Mid-360 TF 架构

ROS2 Humble、Go2、`./scripts/start_go2_nav.sh` 一键栈：

```text
map
 └── odom
      └── base_link
           ├── livox_frame    # Livox 驱动 frame_id，等同雷达链路
           └── imu_link
```

- `map`：先验 PCD / OctoMap 包 / 全局路径。
- `odom`：FAST-LIO 里程计系（`planar_odom_filter` 后供 TF）。
- `base_link`：Go2 机身中心。
- `livox_frame`：Mid-360 点云坐标系（勿写成 `mid360_link`，除非改驱动与 launch）。

---

## 10. 单张跨楼层地图怎么处理

如果你的地图是一张完整的三维地图，而不是每层一张地图，那么建议：

```text
map
 ├── floor_1_area
 ├── stair_area
 ├── floor_2_area
 └── floor_3_area
```

这些区域不一定要做成 TF。

更推荐：

- 所有楼层都在同一个 `map` 坐标系中。
- 楼层高度通过 `z` 轴体现。
- 楼梯、坡道、平台也都在同一张 3D 点云地图里。
- 全局路径直接在同一个 `map` 坐标系里规划。

最简单的 TF 结构仍然是：

```text
map
 └── odom
      └── base_link
```

不要为了每个楼层强行创建：

```text
map -> floor1
map -> floor2
map -> floor3
```

除非你有明确的楼层切换、分区管理或多地图加载需求。

---

## 11. 全局路径和局部路径的坐标系

### 11.1 全局路径

全局路径应当使用：

```text
frame_id = map
```

例如：

```text
global_path.header.frame_id = "map"
```

全局路径表示机器人从当前位置到目标点，在整张地图中的路线。

---

### 11.2 局部目标点

局部控制器可以把全局路径中的一小段转换到：

```text
base_link
```

或者：

```text
odom
```

常见做法：

```text
map 下的全局路径
        ↓ TF 转换
odom / base_link 下的局部目标点
        ↓
局部避障和控制
```

这样局部规划器只需要关心机器人附近一小段路径和障碍物。

---

## 12. 本仓库数据流（一键导航）

```text
/livox/lidar + /livox/imu
        ↓
FAST-LIO2 → /lio_odom_raw、/cloud_registered(odom)
        ↓
planar_odom_filter → /state_estimation
        ↓
tf_manager → odom→base_link
        ↓
Nav2 local_costmap(map) + MPPI → /cmd_vel


先验 PCD + /livox/lidar
        ↓
ICP +（可选）Scan Context + Qt 当前姿态
        ↓
/relocalization/pose → tf_manager → map→odom（锁定）
        ↓
/planned_path(map) + 网页 OctoMap
```

---

## 13. 常见错误结构

### 错误 1：两个模块同时发布 `odom -> base_link`

```text
FAST-LIO2 发布 odom -> base_link
robot_localization 也发布 odom -> base_link
```

问题：

- TF 冲突
- 位姿抖动
- 导航不稳定

解决：

- 只能保留一个模块发布 `odom -> base_link`
- 另一个模块只发布 topic，不发布 TF
- 或者统一由 `robot_localization` 融合后发布 TF

---

### 错误 2：同时存在 `map -> base_link` 和 `odom -> base_link`

```text
map -> base_link
odom -> base_link
```

问题：

- TF 树不唯一
- Nav2 查询 TF 时可能失败
- 局部规划器可能拿到错误位姿

正确：

```text
map -> odom -> base_link
```

---

### 错误 3：全局重定位直接改 `odom -> base_link`

问题：

- 局部控制突然跳变
- 机器人可能急停、抖动、路径错乱

正确：

```text
重定位修正 map -> odom
里程计继续发布 odom -> base_link
```

---

### 错误 4：传感器外参不准

例如：

```text
base_link -> lidar_link
```

外参角度错了 2°，点云地图就可能出现：

- 墙面倾斜
- 地面不平
- 楼梯变形
- 定位漂移
- 障碍物位置偏移

解决：

- 精确测量安装尺寸
- 检查 URDF
- 检查 static TF
- 使用标定工具校准 LiDAR / IMU / Camera 外参

---

## 14. TF 检查命令

### 14.1 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

会生成：

```bash
frames.pdf
```

检查重点：

- 是否只有一条 `map -> odom -> base_link` 链路
- 是否有重复 parent
- 是否有断开的 frame
- 是否有多个模块发布同一个 TF

---

### 14.2 查看某两个坐标系之间的 TF

```bash
ros2 run tf2_ros tf2_echo map base_link
```

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

重点看：

- 是否持续输出
- 时间戳是否更新
- 位姿是否明显跳变
- 数值是否合理

---

### 14.3 查看 TF 话题

```bash
ros2 topic echo /tf
```

```bash
ros2 topic echo /tf_static
```

动态 TF 应该在：

```text
/tf
```

静态 TF 应该在：

```text
/tf_static
```

---

### 14.4 查看是谁在发布 TF

```bash
ros2 topic info /tf -v
```

如果发现多个节点都在发布相同 TF，需要重点检查。

---

## 15. static_transform_publisher 示例

本仓库由 `tf_manager.yaml` 发布 `base_link→livox_frame`（默认 `z=0.15`），一般无需手敲。

若临时调试外参（帧名须与驱动一致 **`livox_frame`**）：

```bash
ros2 run tf2_ros static_transform_publisher \
  0.0 0.0 0.15 \
  0 0 0 \
  base_link \
  livox_frame
```

格式：

```bash
ros2 run tf2_ros static_transform_publisher \
  x y z \
  roll pitch yaw \
  parent_frame \
  child_frame
```

注意：

- 单位是米和弧度。
- parent 通常是 `base_link`。
- child 是传感器坐标系。
- 安装方向错了，点云和地图都会错。

---

## 16. 四足机器人导航推荐模块划分

（本仓库定位层见 **第 8 节、第 0 节**，无 KISS-ICP。）

```text
┌────────────────────────────────────┐
│              全局层                 │
│  PCD Map / OctoMap / dong_path      │
└────────────────────────────────────┘
                  │
                  ▼
┌────────────────────────────────────┐
│              定位层                 │
│  FAST-LIO2 + ICP + Scan Context     │
│  tf_manager: map->odom, odom->base  │
└────────────────────────────────────┘
                  │
                  ▼
┌────────────────────────────────────┐
│              局部规划层             │
│  Local Costmap / Elevation Map      │
│  MPPI / MPC / DWA / 自研避障         │
└────────────────────────────────────┘
                  │
                  ▼
┌────────────────────────────────────┐
│              运动控制层             │
│  四足步态控制 / 速度控制 / 落脚点规划 │
└────────────────────────────────────┘
```

---

## 17. 工程落地建议

### 17.1 TF 职责必须唯一

**本仓库规定**（`go2_nav_stack/config/tf_manager.yaml` + launch 参数）：

| TF | 只允许谁发布 |
|---|---|
| `map -> odom` | **`tf_manager_node`**（`/relocalization/pose`） |
| `odom -> base_link` | **`tf_manager_node`**（`/state_estimation`） |
| `base_link -> livox_frame` | **`tf_manager_node`** 静态 |
| `livox_frame -> imu_link` | **fastlio2 launch** 静态（`publish_base_to_imu: false`） |

FAST-LIO、ICP **不得**再发 `map→odom` 或 `odom→base`（launch 已关）。

---

### 17.2 全局重定位不要破坏局部连续性

重定位成功时，不要直接修改：

```text
odom -> base_link
```

而应该修改：

```text
map -> odom
```

这样局部控制器看到的 `odom -> base_link` 仍然是连续的。

---

### 17.3 局部 costmap 坐标系

**通用 Nav2**：局部 costmap 用 `odom`。

**本仓库**：局部 costmap 用 **`map`**，点云 `/cloud_registered` 在 `odom` 下由障碍层变换进 map。控制器 odom 话题为 `/state_estimation`。重定位只跳 `map→odom`，保持 `odom→base_link` 连续。

---

### 17.4 全局规划必须使用 map

全局规划器推荐使用：

```text
map
```

原因：

- 目标点在全局地图中
- 全局路径需要稳定参考系
- 跨楼层路线需要统一坐标

---

## 18. 本仓库最终配置（Go2 一键栈）

```text
map
 └── odom
      └── base_link
           ├── livox_frame
           └── imu_link
```

地图与话题：

```text
先验 PCD /prior_map              frame_id = map
OctoMap /octomap                   frame_id = map
全局路径 /planned_path             frame_id = map
局部 costmap                       global_frame = map（nav2_mppi_params.yaml）
LIO 点云 /cloud_registered         frame_id = odom
彩色显示 /cloud_registered_map     frame_id = map（Intensity）
ICP 对齐 /transformed_cloud        frame_id = map
里程计 /state_estimation           child_frame_id = base_link，header=odom
```

---

## 19. 一句话总结

> **本仓库**：ICP/Scan Context 重定位后由 `tf_manager` 修正 `map→odom`；FAST-LIO 只出 `/lio_odom_raw`，由 `tf_manager` 发 `odom→base_link`；雷达帧为 `livox_frame`；全局与局部规划均在 `map`；实时彩色点云看 `/cloud_registered_map`（Intensity）。

---

## 20. 本仓库工程检查清单

| 检查项 | 是否必须 | 说明 |
|---|---|---|
| 是否只有 `tf_manager` 发 `map→odom`、`odom→base` | 必须 | FAST-LIO/ICP launch 已关 TF |
| `odom -> base_link` 是否连续 | 必须 | 看 `/state_estimation` |
| 重定位后 `map→odom` 是否锁定 | 必须 | `/relocalization/status=RELOCALIZED` |
| `livox_frame` 与 `tf_manager.yaml` 外参 | 必须 | 勿用 `mid360_link` 除非全改 |
| 全局路径 `/planned_path` 为 `map` | 必须 | dong_path |
| local_costmap `global_frame=map` | 必须 | 与路径同系 |
| RViz Fixed Frame = `map` | 必须 | 否则点云/栅格看似随机身转 |
| `/cloud_registered_map` Intensity 着色 | 推荐 | 需重定位完成 |
| rosbridge 9090 无占用 | 必须 | `stop_go2_nav.sh` |
| 无第二实例 `ros2 launch go2_nav_stack` | 必须 | 防端口与 TF 冲突 |

通用 Nav2 条目（其他项目）：local_costmap 可用 `odom`；本仓库见第 0.4 节。
