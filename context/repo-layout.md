# Repo Layout & Build

- Project root: `/ws`  •  ROS2 workspace root: `/ws/ros_ws`  •  ROS distro: humble.
- `djinn` orchestrator CLI: `/ws/djinn` (bash). Env var `NRT_WS=/ws`.

## Packages of interest (under `/ws/ros_ws/src`)
| Path | Build type | Submodule? |
|------|-----------|------------|
| `src/slam` | ament_cmake (C++) | **YES** — submodule `src/SLAM` → `ros2_orb_slam3.git`, branch `main`. Editing any file here changes the submodule working tree. |
| `src/controllers` | ament_cmake + ament_cmake_python | **No** (regular tracked dir). |
| `src/tello`, `src/mpu6050`, `src/flight_matrix_bridge` | various | submodules |

> **`vision_opencv` / `ros2_shared` are no longer submodules here.** These two fixed upstream build deps were removed from `ros_ws/src` and are now baked into the Docker `ros` image as the `/ros_deps_ws` overlay (built once at image-build time, sourced by the entrypoints and by `scripts/build_ros_packages.sh`). Packages such as `src/sensors`, `src/slam`, `src/tello`, `src/flight_matrix_bridge`, and `src/controllers` still `find_package(cv_bridge/image_geometry/ros2_shared)` — those now resolve from the overlay, not from a local `src/` checkout. See root `context/ros-deps-workspace.md`.

`.gitmodules` maps `src/slam` under submodule name `src/SLAM`.

## Build
- `./scripts/build_ros_packages.sh` (from `/ws/ros_ws` via `djinn build nrt`) — colcon build.
- `src/slam/CMakeLists.txt`: builds executable **`basalt_slam_node`** (line ~171) and
  `orbslam3_mono_node` (line ~101), guarded inside a conditional block (`endif()` ~line 189).
  `basalt_slam_node` links `basalt`, `yaml-cpp`; installed to `lib/slam`.
- `src/controllers/CMakeLists.txt`: `ament_python_install_package(controllers)` installs the
  Python package `controllers`; `install(PROGRAMS scripts/orbslam3_monocular_controller
  DESTINATION lib/controllers)` installs the Python entry-point script. **Any new Python
  entry-point script must be added to this `install(PROGRAMS ...)` list.**

## Basalt data files (`src/slam/ext/basalt/data/`)
- `sitl_calib.json` — EXISTS (used as calibration file for SITL).
- `sitl_config_vo.json` — EXISTS (VO config used by djinn).
- `sitl_config.json` — **does NOT exist** (only referenced as a Python default; harmless because
  the controller only logs it, but launch/djinn should pass `sitl_config_vo.json`).
