# Go2 + ZJU ego-planner-swarm 集成说明

上游仓库：[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)（分支 **`ros2_version`**）

## 1. 同步与编译

```bash
cd ~/dong_3d_nav
bash scripts/sync_ego_planner_swarm.sh
bash scripts/build_ego_planner_swarm_humble.sh
source install/setup.bash
```

官方建议 CycloneDDS（减轻卡顿）：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 2. 启动

```bash
ros2 launch go2_nav_stack bringup_humble.launch.py
```

**不再**随 bringup 启动 Nav2 `controller_server`、MPPI、`/costmap/costmap`（局部障碍由 EGO `grid_map` + `/cloud_registered_map` 处理）。

仅调试旧版 MPPI 时可单独开（与 EGO 不要同时跑，都会发 `/cmd_vel`）：

```bash
ros2 launch go2_nav_stack nav2_mppi.launch.py
```

## 3. 数据流

| 角色 | 话题 / 节点 |
|------|-------------|
| 全局路径 | `dong_path` → `/planned_path`（map） |
| 桥接 | `ego_go2_bridge`：`/lio_odom_raw`（odom 系）→ map 系 `/drone_0_state_estimation`；点云 `/cloud_registered_map` → `/drone_0_cloud_registered_map` |
| 目标 | `ego_path_follower`：`/start_navigation` 后按路径发 `/move_base_simple/goal` |
| 局部规划 | `ego_planner_node` + `traj_server`（B-spline 优化） |
| 速度 | `ego_cmd_bridge`：map 系 `pos_cmd` → `/cmd_vel`（位姿仅用 **TF map→base_link**，不订阅里程计话题） |

**里程计**：默认 **`/lio_odom_raw`**（FAST-LIO2）。`/state_estimation` 为旧链路（`planar_odom_filter` 或 tf_manager 转发），**bringup 默认已不用**。若 RViz 仍订阅 `/state_estimation`，请改 `/lio_odom_raw` 或开 `tf_manager republish_state_estimation:=true`。

## 4. 与已屏蔽的 MPPI / costmap

| 已移除（默认 bringup） | 替代 |
|------------------------|------|
| `nav2_costmap_2d` → `/costmap/costmap` | EGO `grid_map` + `/drone_0_cloud_registered_map` |
| `controller_server` + MPPI | `ego_planner_node` + `traj_server` |
| `mppi_path_follower` | `ego_path_follower` → `/move_base_simple/goal` |
| `nav2_controller_bringup` | 不需要 lifecycle 激活 |

- EGO 为 **3D 轨迹优化**（原为多旋翼），Go2 通过 **cmd 桥接** 压成平面 `vx/wz`。
- 全局路径仍由 **OctoMap A\***（`/planned_path`）生成，EGO 负责局部绕障与跟踪。

## 5. 速度限制（当前默认）

| 层级 | 参数 | 默认值 |
|------|------|--------|
| EGO 轨迹优化 | `ego_planner_core.yaml` → `manager/max_vel`、`optimization/max_vel`、`bspline/limit_vel` | **0.35 m/s** |
| 下发 Go2 | `ego_planner_go2.yaml` → `ego_cmd_bridge/max_vx` | **0.35 m/s** |
| 下发 Go2 | `ego_planner_go2.yaml` → `ego_cmd_bridge/max_wz` | **0.5 rad/s** |

Launch 可覆盖：`ros2 launch go2_nav_stack bringup_humble.launch.py max_vel:=0.35`

## 6. 参数文件与调参（贴障不走）

核心参数在 **`go2_nav_stack/config/ego_planner_core.yaml`**（由 `ego_advanced_param_go2.launch.py` 加载）。

| 参数 | 原 Go2 值 | ZJU 默认 | 现 Go2 值 | 说明 |
|------|-----------|----------|-----------|------|
| `grid_map/obstacles_inflation` | 0.12 | 0.099 | **0.08** | 栅格膨胀；过大通道变窄 |
| `optimization/dist0` | 0.45 | 0.50 | **0.28** | 与障碍安全距离；过大易无解/急停 |
| `optimization/lambda_collision` | 0.50 | 0.50 | **0.35** | 避障权重；过大宁可停也不靠近 |
| `optimization/swarm_clearance` | 0.50 | 0.50 | **0.35** | 群障 clearance |
| `grid_map/p_occ` | 0.80 | 0.80 | **0.85** | 略提高，减轻噪声点云误判占据 |
| `manager/max_vel` / `ego_cmd_bridge max_vx` | **0.35** | 2.0 | — | 直线最大速度 (m/s) |
| `ego_cmd_bridge max_wz` | **0.5** | — | — | 转弯角速度上限 (rad/s) |

仍贴障停：在 RViz 看 `/drone_0_grid/grid_map/occupancy_inflate` 是否把可通行道标满；可再把 `dist0` 降到 `0.25` 或 `obstacles_inflation` 降到 `0.06`（注意安全）。

运行时覆盖速度：

```bash
ros2 launch go2_nav_stack bringup_humble.launch.py max_vel:=0.35
```

## 7. 转向时 ODOM / TF 漂移

**现象**：某一侧转弯时 `map` 里机体或点云“拖影/飘”，另一侧正常。

**常见原因**：
- `map->odom` 用 **now()** 刷新时间戳，而 `odom->base` 用 **LIO 旧 stamp**（~10Hz），转向时两链时间不一致 → `map->base_link` 插值错位。
- `ego_go2_bridge` 曾用手写 `yaw_tf+yaw_o` 合成，且在 TF 失败时误用 `map->base_link` 当 `map->odom`。
- `map_cloud_display` 的 `use_latest_tf:=true` 在转圈时用最新 TF 变换旧点云，障碍相对狗会滑。

**当前策略**（`tf_manager.yaml`）：
- `refresh_map_to_odom_stamp: false`（重定位后 latch，不 50Hz 刷 stamp）
- 转向 **yaw 低通**（`yaw_smooth_alpha` / `yaw_smooth_alpha_spin`）
- `ego_go2_bridge` 只查 **TF map→base_link** 填 `state_estimation`
- `map_cloud_display`：`use_latest_tf: false`，优先点云时间戳

仍飘时可略增大 `yaw_smooth_alpha_spin`（如 0.92），或检查重定位是否用了与 ICP 同一时刻的 odom TF。

## 8. 按全局路径逐点执行（默认）

| 节点 | 行为 |
|------|------|
| **ego_path_follower** | `follow_mode: per_waypoint`（默认）：**一次只盯一个路点** `wp k/N`，到点或超时后 `k→k+1` |
| **ego_planner** | `planning_horizon` **0.8 m**（逐点模式） |
| **ego_cmd_bridge** | `traj_yaw_weight: 0.8`、`waypoint_yaw_weight: 0.2`（行走优先、转弯次之） |

日志：`EGO 逐点 wp 3/56 ...`

可选：`follow_mode: lookahead` + `lookahead_m: 2.0` 沿路线前视，不必踩每个中间点。

**障碍点云高度**（`ego_go2_bridge`）：相对 `base_link`，**激光高度 0.27 m 向下 30 cm 至向上 1.0 m** 内的点送入 EGO 栅格（`z ∈ [-0.03, 1.0]` m），作为不可通行障碍带。

## 9. EGO 路径与全局路径重合 / 原地转圈

**看起来 EGO 路径 = 全局路径**：正常。`ego_path_follower` 把 A* 路点逐个发给 EGO，开阔段局部 B 样条会贴着全局折线；RViz 里 **`/ego_exec_path`** 是 `pos_cmd` 轨迹尾迹，不是优化器输出。看局部差异请开 **`EGO_OptimalTraj`**（`/drone_0_plan_vis/optimal_list`）。

**日志 vx≈0.35 但狗上 echo 为 vx≈0.12、wz≈0.4**：
1. 在**发布端**核对：`ros2 topic echo /cmd_vel --once` 应与 `ego_cmd_bridge` 日志一致。
2. 查是否多发布者：`ros2 topic info /cmd_vel -v`（勿同时开 `d1_controller`、网页 `web_cmd_vel`、MPPI）。
3. 若 echo 一致而狗仍怪，为**机载运动模式/限速**在截断，非桥接节点。

**`wp 57/57` 且 `dist_path=10m`**：折返路径上「几何最近路点」曾误跳到终点；已改为**顺序推进路点**，未走完不发最后一个 wp。

**一上来就先旋转（第一反应）**：
- 不是 EGO 在读全局路径角度，而是 **`ego_cmd_bridge` 曾开启「先对准路点方位」**（`enable_path_yaw_align`）：机体朝向与 1.5 m 前视点方位差 >26° 时先 `wz` 大、`vx` 很小。
- 默认已改为 **`enable_path_yaw_align: false`**：**角速度跟 EGO `pos_cmd.yaw`（轨迹切线）**，线速度沿路点前进，边转边走。
- 若仍想先转再走，在 `ego_planner_go2.yaml` 设 `enable_path_yaw_align: true`。

**持续转圈** 其它原因（已修）：
1. 重复发同一 goal 触发 EGO 重规划 → 仅路点变化时发布（3s 心跳）。
2. EGO `WAIT_TARGET` 且距路点仍远 → 轻转 + 慢前进，等 `goal_heartbeat` 重发。
3. EGO goal z 曾写死 1.0 m → 已用消息 z。

## 10. RViz 显示 EGO 路径

一键启动（`go2_nav_oneclick.sh`）且 `launch_rviz:=true` 时，会自动加载：

`go2_nav_stack/config/rviz_go2_nav.rviz`

| 显示名 | 类型 | 话题 | 含义 |
|--------|------|------|------|
| **GlobalPathViz** | Path | `/planned_path_viz` | 全局 A*（绿，由 `ego_traj_display` 转发，RViz 可稳定显示） |
| **EgoExecPath** | Path | `/ego_exec_path` | EGO 实际跟踪轨迹（橙，来自 `pos_cmd`） |
| **GlobalPathMarker** | Marker | `/planned_path_marker` | 全局路径线（粗） |
| **EGO_OptimalTraj** | Marker | `/drone_0_plan_vis/optimal_list` | EGO 优化轨迹点（需正在规划） |
| **EGO_Goal** | Marker | `/drone_0_plan_vis/goal_point` | EGO 局部目标点 |
| **EGO_OccupancyInflate** | PointCloud2 | `/drone_0_grid/grid_map/occupancy_inflate` | 局部膨胀障碍 |

**Fixed Frame 必须为 `map`**（配置文件已设）。

**路径看不到**：`/planned_path` 为 TRANSIENT_LOCAL，RViz 默认 Volatile 常收不到——请订阅 **`/planned_path_viz`** 或 **`/ego_exec_path`**（`ego_traj_display` 节点）。

**`/drone_0_grid/grid_map/occupancy_inflate` 一整片面都是障碍**：来自 EGO 的 **virtual_ceil（虚拟天花板）**——在局部地图整块 XY 上、固定高度铺一层膨胀障碍（多旋翼遗留）。Go2 已在 `ego_planner_core.yaml` 设 **`virtual_ceil_height: -1.0` 关闭**。

**规划完全局路径后“真障碍没了”**：
- `occupancy_inflate` 每次先**清空局部范围**再按当前点云重建；狗若静止、或点云暂时不更新，只剩历史一帧，墙体会“消失”。
- 看真实环境请用 RViz **`CloudRegisteredMap`**（`/cloud_registered_map`），不要只盯 inflate。
- 开始导航后点云持续输入，inflate 会跟着更新。
- **勿**对旧点云做定时重发：狗移动后 map 系旧帧会让 EGO `occupancy_inflate` 相对机体“不刷新”。`ego_go2_bridge` 只转发 **实时** `/cloud_registered_map`；若 >0.35s 无新帧会告警。

**转弯时 `/cmd_vel` 只有 ~0.03 m/s、日志 `dist_goal≈0.06m`、卡在同一 wp**：
- 旧逻辑用 **pos_cmd 轨迹点** 算距离；局部轨迹末端常在机体前 6 cm，`vx_ff≈0` 且不满足 `min_vx` 条件，狗几乎只微转。
- `ego_cmd_bridge` 现订阅 **`/move_base_simple/goal`**，用路径前视点驱动线速度；日志为 `dist_path` / `dist_cmd`。
- `ego_path_follower`：路径进度单调前进；每周期最多前跳 **1** 个路点（避免 wp 20→24 连跳）；`ego_cmd_bridge` 开启 **vx/wz 耦合**，避免 `vx=0.35` 与 `wz=0.5` 同时打满。

点云高度裁剪（相对 base）：**z ∈ [激光−0.30, +1.0] m**，默认约 **[-0.03, 1.0] m**（雷达 0.27 m）。

开始导航后若看不到 `EGO_OptimalTraj`，先确认：

```bash
ros2 topic hz /drone_0_plan_vis/optimal_list
```

### 手动添加（已开 RViz 时）

1. 左下角 **Fixed Frame** → `map`
2. **Add** → **By topic** → 选 `/drone_0_plan_vis/optimal_list` → **Marker**
3. 再添加 `/planned_path` → **Path**、可选 `/drone_0_grid/grid_map/occupancy_inflate` → **PointCloud2**

或单独打开配置：

```bash
rviz2 -d $(ros2 pkg prefix go2_nav_stack)/share/go2_nav_stack/config/rviz_go2_nav.rviz
```

其它可视化话题（默认关闭）：`/drone_0_plan_vis/global_list`、`/drone_0_plan_vis/a_star_list`、`/drone_0_plan_vis/init_list`。
