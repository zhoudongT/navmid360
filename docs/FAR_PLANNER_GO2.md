# Go2 + CMU FAR Planner

[FAR Planner](https://github.com/MichaelFYang/far_planner)（CMU Fan Yang）基于动态可见性图做**全局路径/路点**规划；在 CMU [Autonomous Exploration Development Environment](https://www.cmu-exploration.com/) 中与 **local_planner（避障 + 跟踪）** 配合使用。

本仓库集成方式：**FAR 替代 EGO** 作为局部/route 规划，由 `far_waypoint_follower` 将 `/way_point` 转为四足 `/cmd_vel`。

## 1. 获取与编译

```bash
cd /home/allen/navmid360
bash scripts/sync_far_planner.sh      # 克隆到 far_planner_ws/，分支 humble-jazzy
bash scripts/build_far_planner_humble.sh
colcon build --packages-select go2_nav_stack --symlink-install
source install/setup.bash
```

依赖：`ros-humble-pcl-*`、`libopencv-dev`（与 EGO 类似）。

## 2. 启动

```bash
# 替代 EGO（定位栈不变）
ros2 launch go2_nav_stack bringup_humble.launch.py local_planner:=far

# 仍用 EGO（默认）
ros2 launch go2_nav_stack bringup_humble.launch.py local_planner:=ego
```

## 3. 话题与数据流

| 话题 | 说明 |
|------|------|
| `/cloud_registered_map` | 输入：`map_cloud_display` 彩色点云 |
| `/terrain_map`, `/terrain_map_ext`, `/registered_scan` | `far_terrain_bridge` 输出（FAR 输入） |
| `/state_estimation` | FAR `odom_world` |
| `/goal_point` | FAR 目标（`far_goal_bridge` 从 `/move_base_simple/goal` 转发） |
| `/way_point` | FAR 下一路点 |
| `/cmd_vel` | `far_waypoint_follower` |
| `/far_exec_path` | 实际行驶轨迹（RViz Path） |
| `/local_planner/status` | `FAR_FOLLOWING` / `GOAL_REACHED` |

**注意**：FAR 不是 EGO 那种 B-spline 局部优化器；室内精细贴墙绕障弱于 EGO，优势在**可见性图全局绕障、未知区域 attemptable 导航**（需 `is_static_env:=false` 与更完整 terrain 栈）。

## 4. 设目标

1. 网页/Qt 仍发 `/move_base_simple/goal` → `far_goal_bridge` → `/goal_point`
2. 或编译 FAR 的 `goalpoint_rviz_plugin` 后，在 RViz 用 **Goalpoint** 工具（见 FAR 官方 README）

重定位完成后发送目标；`/start_navigation` 会打开 `/update_visibility_graph`。

## 5. RViz（`rviz_go2_nav_far.rviz`）

FAR 一键启动会自动加载该配置（**已关闭全部 EGO 图层**）。

```bash
rviz2 -d $(ros2 pkg prefix go2_nav_stack)/share/go2_nav_stack/config/rviz_go2_nav_far.rviz
```

| 显示名 | 话题 | 含义 |
|--------|------|------|
| FAR_GlobalPath | `/viz_path_topic` | FAR 全局路径 |
| **FAR_ViewpointExtend** | `/viz_viewpoint_extend_topic` | **狗头前黄色扇形/射线**（局部视点延伸，需已发目标且 FAR 在规划） |
| FAR_VGraph | `/viz_graph_topic` | 可见性图；`vertex_angle` / `angle_direct` 为节点黄色方向扇形 |
| FAR_Contour | `/viz_contour_topic` | 障碍轮廓 |
| FAR_Waypoint | `/viz_node_topic` | 路点 / 目标 |
| FAR_ExecPath | `/far_exec_path` | 狗实际轨迹（橙） |
| FAR_TerrainObs / FAR_TerrainFree | `/terrain_map` | 障碍 / 可通行（按 intensity） |
| FAR_ScanObs | `/registered_scan` | 局部扫描 |
| GlobalPathViz | `/planned_path_viz` | OctoMap A*（可选对照） |

工具栏 **2D Goal Pose** 发布到 `/move_base_simple/goal`。Fixed Frame 须为 **`map`**。

## 6. 参数

- Go2 桥接：`go2_nav_stack/config/far_planner_go2.yaml`
- FAR 核心：同文件内 `far_planner:` 段（`robot_dim`、`terrain_free_Z` 等）

## 7. 与 OctoMap A* 全局路径的关系

默认 **仍保留** `dong_path` 的 `/planned_path`（`navigation_humble` 层）。FAR 模式下：

- **`far_path_follower`**：`/start_navigation` 后沿 `/planned_path` **前视 2m** 滚动发布 `/goal_point`，FAR 局部绕障仍可用，整体贴近绿线全局路径
- **`far_goal_bridge`**：RViz **2D Goal Pose** 单点目标（不走路径时）
- 障碍间隙：`robot_dim`、`obs_inflate_size`、`far_terrain_bridge/obstacle_xy_inflate_m` 已按 Go2 加大

若仍贴障，可再增大 `robot_dim`（如 0.72）或 `obstacle_xy_inflate_m`（如 0.20）。

## 8. 参考

- FAR Planner: https://github.com/MichaelFYang/far_planner  
- CMU 开发环境: https://www.cmu-exploration.com/  
- Go2 官方 autonomy 栈（含 FAR）: https://github.com/jizhang-cmu/autonomy_stack_go2  
