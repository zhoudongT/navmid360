# Go2 走廊导航 + MPPI 局部避障

沿 OctoMap `/planned_path` 走廊前进，Nav2 MPPI 在走廊内绕障；重定位优先路径附近 submap ICP。

## 一键启动

```bash
source install/setup.bash
./scripts/go2_nav_oneclick.sh --mppi
# 或
LOCAL_PLANNER=mppi ./scripts/go2_nav_oneclick.sh
```

## 数据流

```text
/planned_path (OctoMap A*)
    ↓
global_path_manager  →  /global_path_processed, /local_target_pose, /path_corridor_marker
    ↓
mppi_path_follower   →  FollowPath → MPPI + costmap → /cmd_vel

重定位:
/planned_path + /path_progress
    ↓
scan_context (path_corridor 过滤)
    ↓
/initialpose → icp_node (submap 25m) → /icp_result → tf_manager (map→odom)
```

## 关键参数

| 文件 | 说明 |
|------|------|
| `config/corridor_mppi.yaml` | 路径管理 + MPPI follower |
| `config/nav2_mppi_params.yaml` | MPPI critic 权重、costmap |
| `config/relocalization.yaml` | SC `path_corridor` |
| `dong_octomap/config/icp_go2_nav.yaml` | `use_submap_icp`, `submap_radius_xy_m` |

## RViz 图层（`rviz_go2_nav_mppi.rviz`）

| 显示名 | 话题 | 含义 |
|--------|------|------|
| GlobalPathViz | `/planned_path` | OctoMap 全局路径（绿） |
| MPPI_CorridorPath | `/global_path_processed` | 当前走廊路径段（青） |
| MPPI_LocalPath | `/local_path` | MPPI 跟踪段（橙） |
| MPPI_MPPIGlobalPlan | `/controller_server/transformed_global_plan` | MPPI 内部裁剪路径（黄） |
| MPPI_Corridor | `/path_corridor_marker` | 走廊边界（绿线） |
| MPPI_LocalTarget | `/local_target_pose` | 前视局部目标（黄箭头） |
| MPPI_Costmap | `/costmap/costmap` | 局部障碍栅格 |
| MPPI_ObstacleCloud | `/cloud_registered` | 实时障碍点云（红） |
| MPPI_Trajectories | `/FollowPath/trajectories` | MPPI 采样轨迹（需导航中） |

一键启动 `--mppi` 会自动加载该 RViz 配置。

## 使用步骤

1. 网页/Qt 规划全局路径 → `/planned_path`
2. **重定位完成**（`/relocalization/ok=true`，约 40–60s 后 `controller_server` ACTIVE）
3. 点击 **开始导航** → `/start_navigation`
4. RViz：`GlobalPathViz`（绿）+ `MPPI_LocalPath`（橙）+ `MPPI_Costmap` + `MPPI_ObstacleCloud`

## 排障（无局部路径 / 狗不动）

```bash
ros2 topic echo /local_planner/status --once   # 应为 MPPI_FOLLOWING
ros2 topic echo /nav2_controller/status --once # 应为 ACTIVE
ros2 topic echo /global_path_manager/status --once
ros2 action list | grep follow_path            # 应有 /controller_server/follow_path
ros2 topic hz /cmd_vel
```

| `/local_planner/status` | 含义 |
|-------------------------|------|
| `WAIT_RELOC` | 未完成 ICP 重定位 |
| `WAIT_CONTROLLER` | controller 未 active（等 ~60s 或看 costmap 日志） |
| `PATH_READY` / `MPPI_FOLLOWING` | 正常 |
| `REJECTED` / `ABORTED` | FollowPath 被拒或全碰撞 |

局部路径 `/local_path`：导航前显示 `/planned_path` 附近预览；FollowPath 运行后显示 MPPI 裁剪段。

**注意**：RViz 里粗青色线可能是 **OctoMap 全局 A***（`/planned_path`），只避静态地图，**不会**躲实时点云障碍；紫色/青色 costmap 才是 MPPI 用的动态障碍。若 `global_path_manager` 崩溃，MPPI 会退化为跟整条全局路径。

## 与 EGO/FAR 区别

- **mppi**：严格沿全局路径走廊，MPPI 只做局部绕障（推荐有 A* 路径时）
- **ego**：B-spline 局部优化，可偏离全局路径绕障
- **far**：可见性图全局搜索，目标由 path follower 滚动
