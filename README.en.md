# dong_3d_nav

[中文](./README.md)

A ROS 2 Foxy based 3D navigation system with a Web interface. It targets the Unitree Go2 with Livox Mid-360 and [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) (`ros2` branch) for SLAM, navigation clouds, and odometry.

This directory contains five ROS 2 packages:

- `dong_map_msgs`: custom service interfaces for saving, loading, and exporting navigation map packages.
- `dong_octomap`: OctoMap management package for importing maps in multiple formats, saving/loading map packages, OctoMap visualization, and map editing.
- `octo_planner`: OctoMap based 3D path planning, path tracking control, Web test launch files, and navigation launch files.
- `dong_dwa_local_planner`: optional DWA-style local velocity planning (ported from MoveBase3D `local_planner_3d`; see package description).
- `dong_relocalization`: optional **ICP relocalization** (prior PCD + live cloud, Open3D), decoupled from FAST-LIO core; see README section below.

## Features

- Import PCD point cloud maps as OctoMap.
- Import ROS 2D occupancy grid maps as 3D OctoMap.
- Convert Gazebo `.world` / `.sdf` scenes to OctoMap.
- Save and load OctoMap map packages.
- View and edit OctoMap cells with a Qt/VTK GUI.
- View OctoMap in a Web page, select start/goal points, and run path planning.
- Provide a navigation entry for the Unitree Go2 with Livox Mid-360 + FAST-LIO, plus a standalone Web test entry.

## Introduction Video

- Bilibili: [Open Source ROS 2 Based 3D Navigation System](https://www.bilibili.com/video/BV1jgR9BmELw)
- YouTube: [Open Source ROS 2 Based 3D Navigation System](https://www.youtube.com/watch?v=CepO90mzIeI)

## Directory Layout

```text
dong_3d_nav/
├── dong_map_msgs/        # Custom srv interfaces
├── dong_octomap/         # OctoMap import, management, editing, Web/GUI tools
├── dong_dwa_local_planner/  # Optional DWA local planner (from MoveBase3D)
├── dong_relocalization/     # Optional ICP relocalization (prior PCD + live scan)
├── octo_planner/        # 3D planner, controller, navigation launch files
├── worlds/              # Example Gazebo worlds
└── install_deps_foxy.sh
```

## Requirements

- Ubuntu 20.04
- ROS 2 Foxy
- `colcon`
- OctoMap / octomap_msgs
- OpenCV
- Open3D C++ (`octomap_open3d_viewer_node`, `pcd_to_octomap_node`): Ubuntu 20.04 has no `libopen3d-dev`. Running `install_deps_foxy.sh` installs **LLVM `libc++-dev` / `libc++abi-dev`** (needed to link the prebuilt `libOpen3D.so`) and extracts the official **Open3D 0.17 x86_64 devel** tree to **`~/.local/opt/open3d-0.17.0-cxx11`**. Before building, set  
  `export CMAKE_PREFIX_PATH="$HOME/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"`  
  (the script’s “Next steps” section shows the same.)
- PyQt5, VTK, NumPy, Pillow, PyYAML; map GUI also needs **`ros-foxy-sensor-msgs-py`** (`sensor_msgs_py`, included in `install_deps_foxy.sh`)
- Optional: `rosbridge_server`, used by the Web page to access ROS through websocket

Before running full-stack navigation `octo_planner/launch/nav.launch.py`, add these to the same workspace:

- [`go2_description`](https://github.com/Unitree-Go2-Robot/go2_description) (Go2 URDF; `nav.launch.py` loads `urdf/go2_description.urdf` by default)
- [`unitreerobotics/unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) (Go2 control and networking; [Unitree developer support](https://support.unitree.com/home/en/developer))
- [`Ericsii/FAST_LIO_ROS2`](https://github.com/Ericsii/FAST_LIO_ROS2) (`ros2` branch; `fast_lio` with default `config/mid360.yaml`)
- [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) (Mid-360 point cloud; usually built alongside FAST-LIO)

To clone **FAST_LIO_ROS2** and **livox_ros_driver2** into **this repository root** next to packages like `octo_planner` (as `fast_lio` and `livox_ros_driver2`; ignored by `.gitignore`):

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash scripts/sync_fastlio_mid360_deps.sh
```

If you previously cloned them next to `dong_3d_nav` under `src`, from this repo root you can run: `mv ../fast_lio ../livox_ros_driver2 .` (only when those names are not already present here).

### FAST_LIO_ROS2: ROS 2 distro detection and “localization”

- **CMake / distro**: run **`bash scripts/apply_fast_lio_ros2_patches.sh`** after cloning (also invoked by `sync_fastlio_mid360_deps.sh`) before `colcon build --packages-select fast_lio`. The patch resolves **`FASTLIO_ROS_DISTRO`** from `ROS_DISTRO`, then `/opt/ros/<distro>` entries in `CMAKE_PREFIX_PATH`, else defaults to **foxy**, and fixes `rosidl_*` linkage for Foxy vs newer distros.
- **What “localization” means here**: upstream `fastlio_mapping` is **LIO / incremental mapping**; `map_file_path` is used to **save** maps, not to load a prior map for global localization. For global alignment to a prior map, use **`dong_relocalization` (ICP)** with FAST-LIO (`mid360_localization_lio_icp.launch.py`), or the **Fast-LIO2-Localization** bundle below; other stacks include [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION).
- **Suggested bring-up**: `ros2 launch dong_octomap mid360_localization_lio_icp.launch.py prior_pcd_path:=/path/to/map.pcd` (FAST-LIO params from `dong_octomap/config/mid360_fastlio_localization.yaml`; ICP params still in `dong_relocalization/config/relocalization_icp.yaml`).

### Fast-LIO2-Localization (PolarisXQ, prior map + built-in ICP)

Clone **[PolarisXQ/Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization)** into this repo root.

To download **both** Ericsii Fast-LIO2 (`fast_lio/`) and Fast-LIO2-Localization (clones only; you still pick one `fast_lio` for `colcon build`):

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash scripts/sync_fastlio2_and_localization.sh
```

Polaris-only clone:

```bash
bash scripts/sync_fastlio2_localization.sh
```

- **Mutually exclusive with Ericsii `fast_lio`**: the fork still exports the ROS package name **`fast_lio`**. Do **not** build both Ericsii and Polaris `fast_lio` in the same workspace.
- **Build**: `colcon build --symlink-install --packages-select fast_lio icp_relocalization --paths .../Fast-LIO2-Localization/FAST_LIO .../Fast-LIO2-Localization/icp_relocalization` (the sync script prints a template). The same **ROS distro + rosidl** CMake patch is applied to `FAST_LIO/CMakeLists.txt` automatically.
- **Run**: edit `example.launch.py` (`map_path`) and `FAST_LIO/config/fast_lio_relocalization_param.yaml` (`prior_map_path`, must match) plus initial pose per upstream README; then `ros2 launch dong_octomap mid360_fastlio2_localization.launch.py` (starts Livox, then includes upstream `example.launch.py` with RViz). Set **`FASTLIO2_LOCALIZATION_ROOT`** if the repo is not next to this package tree.

### Livox Mid-360 (`livox_ros_driver2`) troubleshooting

1. **Do not run upstream `./build.sh ROS2` inside `livox_ros_driver2`**: that script `cd ../../` then runs `colcon build`, assuming `ws/src/livox_ros_driver2`. With Livox cloned under **`dong_3d_nav/livox_ros_driver2`**, `../../` can land on your home directory and colcon will crawl unrelated trees (Qt, conda, numpy, …), producing floods of **`python_setup_py` / `SyntaxError`**. Build from the workspace root with **`--paths` pointing at this repo’s livox folder**, or run **`bash scripts/build_livox_ros_driver2.sh ~/your/ros2_ws`** after `source /opt/ros/foxy/setup.bash`.
2. **CMake cannot find `liblivox_lidar_sdk_shared.so` / Livox headers**: install [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) per upstream (`sudo make install` so `/usr/local/lib` and includes are populated).
3. **`Init lds lidar fail!`**: invalid path to `MID360_config.json`, bad JSON, or SDK/driver mismatch. After editing JSON, rebuild and `source install` if not using symlink install. Prefer `ros2 launch dong_octomap mid360_livox_driver.launch.py` (absolute default path to the JSON in share); override with `livox_user_config_path:=...` if needed.
4. **Networking**: `host_net_info` IPs must match the **NIC** that actually talks to the lidar; `lidar_configs` `ip` must be the **lidar’s** IP. Ensure L2 connectivity and UDP ports (often ~56100–56501) are not blocked by firewall.
5. **No cloud / IMU**: keep `xfer_format:=1` (CustomMsg) to match FAST-LIO `mid360.yaml` (`lidar_type: 1`); check `ros2 topic echo /livox/lidar`.
6. **`livox_broadcast_code`**: try your device broadcast code via `livox_broadcast_code:=...` if the default placeholder fails discovery.

You can `colcon build` the three packages in this repo without cloning the above; full navigation needs **FAST_LIO_ROS2** (default `mid360.yaml`) and **livox_ros_driver2** in the same workspace, plus `unitree_ros2` for `/cmd_vel` and TF. `nav.launch.py` no longer depends on AgiBot `d1_bringup` / `d1_core`.

## Install Dependencies

Use the helper script:

```bash
cd ~/ros2_ws/src/dong_3d_nav
bash install_deps_foxy.sh
```

If CMake cannot find Open3D, run the dependency script and export `CMAKE_PREFIX_PATH` as above, or install another Open3D C++ tree that provides `Open3DConfig.cmake`.

## Build

If **Miniconda / Anaconda** is installed, **`conda deactivate`** until your prompt shows no `(base)` (or other env) before building; otherwise CMake may pick conda’s Python (e.g. 3.13), which breaks Foxy’s ROSIDL / `empy` and fails while generating `dong_map_msgs`. The `install_deps_foxy.sh` script strips conda from `PATH` for its own run; for **`colcon build`** in your own shell you still need to leave conda off or pass the system interpreter (below).

From the ROS 2 workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If you did **not** install Open3D under `~/.local/opt/open3d-0.17.0-cxx11`, do **not** keep a stale `CMAKE_PREFIX_PATH` pointing at a missing directory (run `unset CMAKE_PREFIX_PATH` first) or colcon will warn that the path does not exist.

If the source directory has been moved, old CMake cache may still point to an old path. Clean the cache and rebuild:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select dong_map_msgs dong_octomap octo_planner \
  --cmake-clean-cache --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Map Import

### Import a PCD Point Cloud Map

```bash
ros2 launch dong_octomap import_pcd_map.launch.py
```

`pcd_to_octomap_node` uses an in-tree **minimal PCD reader** (ASCII / binary / binary_compressed with zlib) and **does not require Open3D C++**. Open3D C++ is only for optional tools such as `octomap_open3d_viewer_node`.

This launch starts:

- `pcd_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `pcd_map_import_gui`
- `octo_planner/dong_path_node`

### Import a ROS 2D Occupancy Grid Map

```bash
ros2 launch dong_octomap import_ros_map.launch.py
```

This launch starts:

- `occupancy_grid_to_octomap_node`
- `octomap_to_occupied_markers_node`
- `map_package_manager`
- `ros_map_import_gui`
- `octo_planner/dong_path_node`

### Import a Gazebo World / SDF

For built-in example worlds, use `world_name`:

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=field.world
```

For an external world file, use an absolute path:

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_file:=/absolute/path/to/map.world
```

If both `world_file` and `world_name` are provided, `world_file` takes precedence.

The `dong_octomap/worlds/` directory provides two example world files. They are installed with the `dong_octomap` package to `share/dong_octomap/worlds/`:

- `2_storey.world`: two-storey building/floor example.
- `field.world`: field example.

Load the two-storey example:

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=2_storey.world
```

Load the field example:

```bash
ros2 launch dong_octomap import_gazebo_world.launch.py world_name:=field.world
```

This launch starts:

- `world_to_octomap_node`
- `world_selector_gui.py`
- `map_package_manager`
- `octo_planner/dong_path_node`

## Map Management and Editing

Main entry for OctoMap management and editing:

```bash
ros2 launch dong_octomap map_manager.launch.py
```

This launch starts:

- `map_package_manager`
- `octomap_to_occupied_markers_node`
- `map_viewer_gui`
- Optional `octo_planner/dong_path_node`

`map_viewer_gui` supports:

- Open map packages.
- Refresh maps.
- Save maps.
- View occupied, preblocked, traversable, and risk-cost layers.
- Edit cells as `occupied`, `preblocked`, `traversable`, or `clear`.
- Select start points, goal points, and navigation targets.

## Web Visualization

### Load a Map and Start the Web Page

```bash
ros2 launch dong_octomap web_octomap.launch.py map_package:=~/maps/map
```

Common parameters:

- `map_package`: saved map package directory.
- `http_port`: static Web server port, default `8080`.
- `launch_rosbridge`: whether to start `rosbridge_websocket`.
- `launch_map_gui`: whether to also start the Qt save/load window.

Open in a browser:

```text
http://<robot-ip>:8080
```

### Web Function Test

```bash
ros2 launch octo_planner web_test.launch.py
```

`web_test.launch.py` is used to test Web access, map display, Web start/goal selection, path planning, and the basic control chain. This launch does not require `go2_description`; it uses a minimal `base_link` URDF for `robot_state_publisher`.

Before starting, configure:

```text
octo_planner/config/nav_params.yaml
```

At minimum, set:

- `relocalization_bin_file`: `.bin` map file used for relocalization.
- `map_package_dir`: saved OctoMap map package directory.

## Full Navigation on Unitree Go2 + Livox Mid-360

Full robot navigation entry:

```bash
ros2 launch octo_planner nav.launch.py
```

This launch targets the Unitree Go2 with **Livox Mid-360 + FAST-LIO** (`fast_lio` `mapping.launch.py`, default `mid360.yaml`) for point clouds and odometry, plus the OctoMap / Web planning stack. It starts or uses:

- `go2_description/urdf/go2_description.urdf` (loaded by `robot_state_publisher`)
- `fast_lio` (build FAST_LIO_ROS2 in your workspace)
- `octo_planner/dong_path_node`
- `octo_planner/d1_controller`
- `dong_octomap/map_package_manager`
- Web viewer and `rosbridge_websocket`

Before running, update the environment-specific configuration:

```text
octo_planner/config/nav_params.yaml
```

Important fields:

- `relocalization_bin_file`
- `map_package_dir`
- `relocalization_pcd_file`
- `show_rviz`
- `show_map_gui`
- `use_static_odom_to_base`

Path tracking defaults to the FAST-LIO **`body`** frame (override `robot_center_offset_frame` under `d1_controller` in `nav_params.yaml` if needed). If OctoMap stays in `map`, publish consistent static TF between **`map`** and **`camera_init`** (see FAST-LIO docs) so RViz and planning frames line up.

**Planning stack:** `dong_path_node` is the **global planner** (A* on the OctoMap grid, `/planned_path`). `d1_controller` is a **path-tracking controller** (geometry-based `cmd_vel` along the path after `/start_navigation`); it is **not** a Nav2-style **local planner** (no local costmap, no dynamic replanning, no DWA/TEB). Use `launch_controller` in `nav.launch.py` / `web_test.launch.py` to enable or disable `d1_controller`.

**Optional DWA:** package `dong_dwa_local_planner` ports the velocity sampling / trajectory scoring from [MoveBase3D](https://github.com/WilsonGuo/MoveBase3D) `local_planner_3d` (ROS2 script upstream). It subscribes to `/planned_path` and an optional obstacle `sensor_msgs/PointCloud2`, publishes `/cmd_vel_dwa` by default (`enabled` defaults to false). Run `ros2 launch dong_dwa_local_planner dwa_local_planner.launch.py` after `source install/setup.bash`.

**Optional ICP relocalization:** package `dong_relocalization` provides `icp_relocalizer_node`: load a prior PCD in `map`, subscribe to a live cloud (default `/cloud_registered`), call service `/icp_relocalizer/trigger_icp` (`std_srvs/Trigger`) when the TF guess is roughly correct; it publishes `/relocalization/lidar_pose_in_map` and does **not** publish TF by default (`publish_tf: false`). Requires `pip install open3d`. Launch: `ros2 launch dong_relocalization relocalization_icp.launch.py` after editing `prior_pcd_path` in the YAML.

**Mid-360 driver + FAST-LIO + ICP in one launch:** `ros2 launch dong_octomap mid360_relocalization_icp.launch.py prior_pcd_path:=/absolute/path/to/map.pcd` starts the Livox node via `dong_octomap/mid360_livox_driver.launch.py` (absolute `MID360_config.json` path), `fast_lio` `mapping.launch.py` (RViz off), and `icp_relocalizer_node`; configure `MID360_config.json` for your sensor first.

In `octo_planner/config/nav_params.yaml`, at minimum configure:

- `relocalization_bin_file`: `.bin` map file used for relocalization.
- `map_package_dir`: saved OctoMap map package directory.

If RViz is needed to observe localization, also configure:

- `relocalization_pcd_file`: `.pcd` point cloud map file for RViz display.

## Other Launch Files

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

`mid360_livox_driver.launch.py` starts only the Mid-360 driver (good for isolating network/topic issues). `mid360_slam.launch.py` and `mid360_loc.launch.py` **include** `fast_lio/launch/mapping.launch.py` (default `mid360.yaml`) and add `dong_octomap` RViz and helper nodes. `mid360_relocalization_icp.launch.py` additionally starts `mid360_livox_driver.launch.py` and `dong_relocalization` ICP (see above). **`mid360_localization_lio_icp.launch.py`** is similar but drives FAST-LIO with **`mid360_fastlio_localization.yaml`** (LIO tuned for working alongside prior-map ICP). **`mid360_fastlio2_localization.launch.py`** requires [Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization) (exclusive with Ericsii `fast_lio`), starts Livox, then includes upstream `example.launch.py`.

## Recommended Starter Book

Robot Operating System ROS2: Introduction and Practice

<img src="./media/book_1.jpg" alt="Robot Operating System ROS2: Introduction and Practice" width="280">

Taobao link: [Robot Operating System ROS2: Introduction and Practice](https://world.taobao.com/item/820988259242.htm)
