# 四足机器狗单张全局地图跨楼层导航工程方案  
## ROS2 Humble + MID360 + FAST-LIO2 + Scan Context + 单地图多楼层导航

版本：v1.1  
适用场景：四足机器狗 / ROS2 Humble / MID360 / FAST-LIO2 / Scan Context / 单张全局地图 / 多楼层导航 / 已有全局路径  
系统环境：Ubuntu 22.04 + ROS2 Humble  

---

## 1. 你的实际情况

你的地图不是每一层一张地图，而是：

```text
整栋建筑 / 多个楼层 / 楼梯 / 走廊 / 电梯区域
全部在同一张 3D 全局点云地图里面
```

所以系统不需要这样做：

```text
floor_1/global_map.pcd
floor_2/global_map.pcd
floor_3/global_map.pcd
```

而应该这样做：

```text
scans_0.5m.pcd
building_semantic.yaml
building_topology_graph.yaml
scan_context_db.bin
floor_zones.yaml
```

核心变化：

```text
不是切换地图
而是在同一张地图里切换 floor_id / navigation_layer / topology node
```

---

## 2. 推荐环境

你的环境是 ROS2 Humble，推荐组合：

```text
Ubuntu 22.04
ROS2 Humble
MID360
livox_ros_driver2
FAST_LIO_ROS2
Scan Context
ICP / NDT
自研 path_follower
自研 local_planner
自研 tf_manager_node
```

---

## 3. 推荐 GitHub 路径

### 3.1 MID360 驱动

```bash
https://github.com/Livox-SDK/livox_ros_driver2
```

### 3.2 ROS2 FAST-LIO2

推荐：

```bash
https://github.com/Ericsii/FAST_LIO_ROS2
```

### 3.3 官方 FAST-LIO 原版参考

```bash
https://github.com/hku-mars/FAST_LIO
```

### 3.4 Scan Context

```bash
https://github.com/gisbi-kim/scancontext_tro
```

---

## 4. 单张地图的正确架构

原来多地图方案是：

```text
floor_1 map
floor_2 map
floor_3 map
```

你的方案应该改成：

```text
一张 building_global_map.pcd
        ↓
按高度 Z / 拓扑区域 / 语义节点区分楼层
        ↓
全局路径在同一地图坐标系下生成
        ↓
局部规划器根据当前 floor_id 和局部点云避障
```

系统结构：

```text
MID360 + IMU
    ↓
FAST-LIO2
    ↓
odom -> base_link
    ↓
Scan Context + ICP/NDT
    ↓
map -> odom
    ↓
single_map_manager
    ↓
floor_zone_manager
    ↓
path_follower
    ↓
local_planner
    ↓
robot_motion_bridge
```

---

## 5. TF 树设计

单张地图时，TF 树更简单：

```text
map
 └── odom
      └── base_link
           ├── imu_link
           └── livox_frame
```

注意：

```text
map 永远是一张全局地图的坐标系
不要因为切楼层而改变 map frame
不要使用 map_floor1 / map_floor2 这种 frame
```

推荐：

| TF | 发布者 | 类型 |
|---|---|---|
| `map -> odom` | relocalization_manager / tf_manager_node | 动态 |
| `odom -> base_link` | FAST-LIO2 或 tf_manager_node | 动态 |
| `base_link -> imu_link` | static_transform_publisher | 静态 |
| `base_link -> livox_frame` | static_transform_publisher | 静态 |

---

## 6. 为什么单张地图更适合你

优点：

```text
1. 不需要切换 PCD 地图
2. 不需要切换 map frame
3. 楼梯、电梯、斜坡、楼层连接关系天然连续
4. 全局路径可以直接跨楼层生成
5. FAST-LIO2 的坐标系更容易保持连续
6. TF 更不容易断
```

缺点：

```text
1. 地图比较大
2. Scan Context 搜索范围更大
3. 楼层相似时容易误匹配
4. 需要增加 floor_id 判断
5. 需要对点云做高度/区域分区
```

所以你要补的是：

```text
单地图楼层语义管理
```

不是：

```text
多地图切换管理
```

---

## 7. 单地图文件结构建议

建议目录：

```text
maps/
 ├── building_global_map.pcd
 ├── building_downsampled_map.pcd
 ├── scan_context_db.bin
 ├── keyframes/
 │    ├── keyframe_000001.pcd
 │    ├── keyframe_000002.pcd
 │    └── ...
 ├── keyframe_poses.csv
 ├── floor_zones.yaml
 ├── building_topology_graph.yaml
 ├── semantic_nodes.yaml
 └── navigation_layers/
      ├── floor_1_nav.yaml
      ├── floor_2_nav.yaml
      └── floor_3_nav.yaml
```

---

## 8. floor_zones.yaml 示例

用 Z 高度和区域定义楼层：

```yaml
floors:
  - id: F1
    name: floor_1
    z_min: -0.5
    z_max: 2.8

  - id: F2
    name: floor_2
    z_min: 2.9
    z_max: 6.0

  - id: F3
    name: floor_3
    z_min: 6.1
    z_max: 9.2

connectors:
  - id: stair_A
    type: stair
    from: F1
    to: F2
    region:
      min: [10.0, 3.0, -0.5]
      max: [14.0, 6.0, 6.0]

  - id: elevator_A
    type: elevator
    from: F1
    to: F2
    region:
      min: [20.0, 10.0, -0.5]
      max: [23.0, 13.0, 6.0]
```

注意：

```text
如果楼层不是标准水平面，比如坡道/楼梯，需要给 connector 单独定义区域。
```

---

## 9. building_topology_graph.yaml 示例

全局拓扑不按地图分，而按节点分：

```yaml
nodes:
  - id: F1_START
    floor: F1
    xyz: [1.0, 2.0, 0.0]

  - id: F1_STAIR_A_IN
    floor: F1
    xyz: [10.0, 4.0, 0.0]

  - id: F2_STAIR_A_OUT
    floor: F2
    xyz: [10.5, 4.2, 3.2]

  - id: F2_TARGET
    floor: F2
    xyz: [30.0, 8.0, 3.2]

edges:
  - from: F1_START
    to: F1_STAIR_A_IN
    type: normal

  - from: F1_STAIR_A_IN
    to: F2_STAIR_A_OUT
    type: stair

  - from: F2_STAIR_A_OUT
    to: F2_TARGET
    type: normal
```

你的全局路径可以直接是：

```text
[x, y, z, yaw, floor_id, node_id]
```

不要只保存 x/y。

---

## 10. 单张地图下的楼层判断

当前楼层可以通过三种方式判断：

### 10.1 根据 Z 高度判断

```text
current_z 落在哪个 floor_zones 的 z_min / z_max 之间
```

优点：

```text
简单
```

缺点：

```text
楼梯、电梯、坡道区域容易误判
```

---

### 10.2 根据拓扑状态判断

例如：

```text
机器人正在 stair_A 区域
当前状态是 TAKE_STAIRS
则允许 floor_id 从 F1 变成 F2
```

优点：

```text
不会因为 Z 抖动频繁切层
```

---

### 10.3 根据 Scan Context / ICP 匹配关键帧判断

每个关键帧保存：

```text
keyframe_id
pose
floor_id
node_id
scan_context_descriptor
```

重定位成功后，根据候选关键帧的 floor_id 更新当前楼层。

这是最稳的方式。

---

## 11. 推荐楼层判断逻辑

不要只用 Z。

推荐融合：

```text
current_floor = f(z_height, topology_region, scan_context_candidate)
```

逻辑：

```text
1. 普通区域：优先用 Z 判断
2. 楼梯/电梯区域：优先用状态机判断
3. 重定位成功：以 Scan Context 候选关键帧 floor_id 为准
4. 如果结果冲突：机器人停止，进入 RELOCALIZATION
```

---

## 12. FAST-LIO2 的职责

FAST-LIO2 只做：

```text
实时里程计
odom -> base_link
/cloud_registered
/path
```

不要让 FAST-LIO2 做：

```text
楼层判断
全局路径规划
局部避障
电梯状态管理
地图切换
```

在你的单地图方案里，FAST-LIO2 仍然只负责连续 odom。

---

## 13. Scan Context 在单地图中的使用方式

多地图时：

```text
每层一个 Scan Context DB
```

你的单地图时：

```text
整栋楼一个 Scan Context DB
每个 keyframe 带 floor_id / node_id / z / region_id
```

keyframe 数据示例：

```csv
keyframe_id,x,y,z,roll,pitch,yaw,floor_id,node_id,region_id
000001,1.0,2.0,0.0,0,0,0,F1,F1_START,normal
000230,10.0,4.0,0.0,0,0,1.57,F1,F1_STAIR_A_IN,stair_A
000450,10.5,4.2,3.2,0,0,1.57,F2,F2_STAIR_A_OUT,stair_A
```

---

## 14. 开机定位流程

```text
机器人开机
  ↓
采集 3~5 秒点云
  ↓
生成当前 Scan Context
  ↓
在整栋楼 scan_context_db.bin 里搜索 Top-K
  ↓
得到候选 keyframe
  ↓
根据候选 keyframe 获取：
      floor_id
      node_id
      粗略 pose
  ↓
ICP / NDT 精配准
  ↓
验证成功
  ↓
设置 map -> odom
  ↓
设置 current_floor
  ↓
进入导航
```

注意：

```text
单张地图时不需要加载 floor_2_map
只需要更新 current_floor 和当前导航层
```

---

## 15. 重定位流程

```text
定位异常
  ↓
机器人停止
  ↓
采集当前点云
  ↓
Scan Context 在整栋楼 keyframe DB 中搜索
  ↓
得到 Top-K 候选
  ↓
ICP/NDT 对 building_global_map.pcd 或局部 submap 精配准
  ↓
成功后更新 map -> odom
  ↓
恢复导航
```

建议不要直接对完整大地图做 ICP。

推荐：

```text
Scan Context 找到 keyframe
  ↓
根据 keyframe 附近 10~30m 截取 submap
  ↓
当前点云和 submap 做 ICP/NDT
```

这样速度更快，也更稳定。

---

## 16. 单地图下的全局路径格式

你已经有全局路径，建议格式改成：

```yaml
path:
  - seq: 0
    xyz: [1.0, 2.0, 0.0]
    yaw: 0.0
    floor: F1
    node: F1_START
    type: normal

  - seq: 1
    xyz: [10.0, 4.0, 0.0]
    yaw: 1.57
    floor: F1
    node: F1_STAIR_A_IN
    type: stair_in

  - seq: 2
    xyz: [10.5, 4.2, 3.2]
    yaw: 1.57
    floor: F2
    node: F2_STAIR_A_OUT
    type: stair_out

  - seq: 3
    xyz: [30.0, 8.0, 3.2]
    yaw: 0.0
    floor: F2
    node: F2_TARGET
    type: normal
```

重点：

```text
路径点必须带 z 和 floor_id
否则跨楼层时局部规划器不知道当前走哪一层
```

---

## 17. 已经有全局路径后，需要补充的模块

你现在不缺全局规划，缺的是执行层。

必须补：

```text
1. path_follower
2. floor_zone_manager
3. local_map_builder
4. local_planner
5. path_tracker
6. robot_motion_bridge
7. recovery_manager
8. navigation_state_machine
```

---

## 18. path_follower 设计

职责：

```text
1. 接收完整 global_path
2. 根据 current_pose 找最近路径点
3. 根据 lookahead_distance 取前方 local_goal
4. 根据路径点 floor_id 判断当前导航层
5. 遇到 stair/elevator 节点时切换状态机
```

输入：

```text
/global_path
/current_pose
/current_floor
```

输出：

```text
/local_goal
/local_path_segment
/current_target_floor
/current_target_node
```

---

## 19. local_map_builder 设计

输入：

```text
MID360 当前点云
FAST-LIO2 当前位姿
building_global_map.pcd
current_floor
local_goal
```

输出：

```text
local_costmap
elevation_map
traversability_map
```

单张地图时，需要从全局地图中截取局部区域：

```text
以机器人当前位置为中心
截取半径 5~10m 的 submap
再叠加实时点云动态障碍
```

推荐：

```text
static_submap = 从 building_global_map.pcd 截取
dynamic_obstacles = 当前实时点云检测
local_costmap = static_submap + dynamic_obstacles
```

---

## 20. local_planner 设计

局部规划器目标：

```text
沿全局路径走
局部绕障
绕完回到全局路径
```

推荐算法：

第一阶段：

```text
局部 A*
```

第二阶段：

```text
D* Lite
```

动态障碍多时：

```text
D* Lite 更适合频繁局部更新
```

---

## 21. 四足可通行性

四足不能只看 2D costmap。

需要判断：

```text
1. 脚能不能踩
2. 身体会不会撞
3. 高度差能不能过
4. 坡度能不能走
5. 楼梯区域是否进入特殊步态
6. 窄通道是否够身体通过
```

推荐 2.5D elevation map：

```text
每个 grid 记录：
height
slope
roughness
obstacle_height
traversability_score
```

---

## 22. 状态机设计

单张地图下，状态机不切地图，只切状态和 floor_id。

```text
IDLE
 ↓
RELOCALIZATION_ON_START
 ↓
GLOBAL_PATH_READY
 ↓
LOCAL_NAVIGATION
 ↓
APPROACH_CONNECTOR
 ↓
TAKE_STAIRS / TAKE_ELEVATOR
 ↓
FLOOR_ID_UPDATE
 ↓
RELOCALIZATION_AFTER_CONNECTOR
 ↓
CONTINUE_NAVIGATION
 ↓
ARRIVED
```

异常状态：

```text
TF_LOST
LOCALIZATION_LOST
PATH_BLOCKED
FLOOR_MISMATCH
RELOCALIZATION_FAILED
EMERGENCY_STOP
```

---

## 23. 跨楼层流程：楼梯

```text
机器人在 F1
  ↓
沿全局路径到 F1_STAIR_A_IN
  ↓
状态切换为 TAKE_STAIRS
  ↓
局部规划器切换 stair mode
  ↓
允许 z 连续变化
  ↓
通过楼梯区域
  ↓
到达 F2_STAIR_A_OUT
  ↓
更新 current_floor = F2
  ↓
触发一次 Scan Context / ICP 重定位
  ↓
继续执行 F2 路径
```

楼梯区域不要简单用普通 2D costmap。

要有：

```text
stair mode
低速
更大安全边界
允许连续 z 变化
限制横向速度
```

---

## 24. 跨楼层流程：电梯

```text
机器人在 F1
  ↓
导航到 F1_ELEVATOR_IN
  ↓
进入电梯
  ↓
停止运动
  ↓
等待电梯到 F2
  ↓
出电梯
  ↓
current_floor = F2
  ↓
Scan Context + ICP 重定位
  ↓
继续导航
```

单张地图下仍然需要重定位，因为：

```text
电梯内部 LiDAR 特征少
垂直移动过程中 FAST-LIO2 可能漂
出电梯后必须重新确认 map -> odom
```

---

## 25. TF 不丢的工程方案

### 25.1 TF 树固定

```text
map -> odom -> base_link -> livox_frame
map -> odom -> base_link -> imu_link
```

### 25.2 不允许多个节点抢发同一个 TF

只允许：

```text
tf_manager_node 发布 map -> odom
tf_manager_node 或 FAST-LIO2 发布 odom -> base_link
static_transform_publisher 发布 base_link -> livox_frame
static_transform_publisher 发布 base_link -> imu_link
```

### 25.3 建议统一由 tf_manager_node 发布动态 TF

更稳的方式：

```text
FAST-LIO2 只输出 /odom
tf_manager_node 读取 /odom
tf_manager_node 发布 odom -> base_link
tf_manager_node 发布 map -> odom
```

---

## 26. ROS2 Humble launch 结构

```text
bringup_humble.launch.py
 ├── livox_ros_driver2
 ├── fast_lio2
 ├── static_tf_base_to_imu
 ├── static_tf_base_to_livox
 ├── tf_manager_node
 ├── scan_context_manager
 ├── relocalization_manager
 ├── single_map_manager
 ├── floor_zone_manager
 ├── global_path_server
 ├── path_follower
 ├── local_map_builder
 ├── local_planner
 ├── path_tracker
 ├── robot_motion_bridge
 └── navigation_state_machine
```

---

## 27. 参数文件建议

### 27.1 frame_config.yaml

```yaml
frames:
  map_frame: map
  odom_frame: odom
  base_frame: base_link
  imu_frame: imu_link
  lidar_frame: livox_frame
```

---

### 27.2 single_map_manager.yaml

```yaml
single_map_manager:
  global_map: maps/building_global_map.pcd
  downsampled_map: maps/building_downsampled_map.pcd
  scan_context_db: maps/scan_context_db.bin
  keyframe_pose_file: maps/keyframe_poses.csv
  floor_zone_file: maps/floor_zones.yaml
  topology_graph_file: maps/building_topology_graph.yaml

  submap:
    radius: 15.0
    voxel_leaf_size: 0.1
```

---

### 27.3 floor_zone_manager.yaml

```yaml
floor_zone_manager:
  use_z_height: true
  use_topology_region: true
  use_scan_context_floor: true

  floor_switch:
    min_confirm_time: 2.0
    min_z_gap: 1.0
    allow_switch_only_in_connector: true
```

---

### 27.4 relocalization.yaml

```yaml
relocalization:
  search_scope: whole_building
  top_k_candidates: 10

  scan_context:
    min_similarity_score: 0.15

  submap_icp:
    enable: true
    submap_radius: 20.0
    max_iteration: 50
    max_correspondence_distance: 1.0
    fitness_score_threshold: 0.5

  update_tf:
    update_map_to_odom: true
    never_jump_odom_to_base: true
```

---

### 27.5 local_planner.yaml

```yaml
local_planner:
  planner_type: dstar_lite

  local_map:
    length: 8.0
    width: 8.0
    height_filter_by_floor: true
    resolution: 0.05

  path_following:
    lookahead_distance: 1.5
    max_path_deviation: 2.0
    goal_tolerance: 0.3

  stair_mode:
    enable: true
    max_vx: 0.2
    max_vy: 0.05
    max_wz: 0.2
```

---

### 27.6 tf_manager.yaml

```yaml
tf_manager:
  publish_rate: 50.0
  tf_timeout: 0.3

  frames:
    map: map
    odom: odom
    base_link: base_link
    imu_link: imu_link
    lidar_frame: livox_frame

  static_tf:
    base_to_imu:
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]

    base_to_lidar:
      xyz: [0.10, 0.00, 0.20]
      rpy: [0.0, 0.0, 0.0]
```

---

## 28. 调试命令

### 28.1 检查 TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo base_link imu_link
```

### 28.2 检查 topic

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
ros2 topic hz /cloud_registered
ros2 topic hz /tf
```

### 28.3 检查坐标

```bash
ros2 topic echo /odom
ros2 topic echo /current_floor
ros2 topic echo /local_goal
ros2 topic echo /planner_status
```

---

## 29. 单地图方案的开发优先级

推荐你按这个顺序做：

```text
1. Humble + livox_ros_driver2 跑通 MID360
2. FAST_LIO_ROS2 跑通 odom 和 cloud_registered
3. 建立固定 TF 树
4. 编写 tf_manager_node
5. 加载 building_global_map.pcd
6. 建立 keyframe_poses.csv
7. 生成 scan_context_db.bin
8. 实现 Scan Context 开机定位
9. 实现 submap ICP/NDT 精配准
10. 实现 floor_zone_manager
11. 接入已有 global_path
12. 实现 path_follower
13. 实现 local_map_builder
14. 实现 local_planner
15. 实现 robot_motion_bridge
16. 实现 navigation_state_machine
17. 测试楼梯/电梯跨层
```

---

## 30. 最小可运行版本 MVP

你现在最小可运行版本应该是：

```text
livox_ros_driver2
  ↓
FAST_LIO_ROS2
  ↓
tf_manager_node
  ↓
single_map_manager
  ↓
path_follower
  ↓
local_map_builder
  ↓
local_planner
  ↓
path_tracker
  ↓
robot_motion_bridge
```

第一阶段可以先不做完整 Scan Context，只做：

```text
手动给初始位姿
验证 TF
验证路径跟踪
验证局部避障
```

第二阶段再做：

```text
Scan Context 自动开机定位
丢定位恢复
跨楼层后重定位
```

---

## 31. 关键结论

你的方案应该从：

```text
多楼层多地图切换
```

改成：

```text
单张全局 3D 地图 + floor_id + topology graph + semantic zones
```

也就是：

```text
地图不切
frame 不切
只切 current_floor、导航层、状态机和局部规划模式
```

最终核心架构：

```text
FAST-LIO2 负责连续 odom
Scan Context 负责在整张地图中重定位
ICP/NDT 负责精配准
tf_manager_node 负责 TF 不丢
single_map_manager 负责加载单张全局地图
floor_zone_manager 负责判断当前楼层
path_follower 负责执行已有全局路径
local_planner 负责局部绕障
navigation_state_machine 负责跨楼层流程
robot_motion_bridge 负责控制机器狗
```

最重要的一句话：

```text
你的地图是一张，所以不要做“地图切换系统”。
你要做的是“单地图多楼层语义导航系统”。
```
