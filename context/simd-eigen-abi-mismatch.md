# Eigen/`-march` ABI mismatch between `libbasalt.so` and `basalt_slam_node`

**Date:** 2026-07-24, revised 2026-07-25. **Status:** root-caused and **fixed** 2026-07-25; see [`/ws/plans/basalt-slam-defect-remediation.md`](../../plans/basalt-slam-defect-remediation.md) Phase 1 for the change-record. Awaiting the container-side rebuild that confirms the layouts now agree.
**Full tutorial (SIMD, `-march`, Eigen alignment, debugging method):** [`/ws/ros_ws/src/slam/context/SIMD.md`](../src/slam/context/SIMD.md). **Package-level build/ABI context:** [`/ws/ros_ws/src/slam/context/build-and-abi.md`](../src/slam/context/build-and-abi.md).

## Symptom

`basalt_slam_node` takes `SIGSEGV` inside `/lib64/ld-linux-x86-64.so.2` during `BasaltSLAMNode::on_activate` (`src/basalt/node.cpp:98`). The backtrace shows `tf2_ros::TransformBroadcaster` → `rclcpp::create_publisher` → `rmw_create_publisher` → `rcutils_load_shared_library` → `dlopen` → eleven frames of loader internals. The last log lines are "Creating SLAM Object in Basalt Slam Node" then "Creating SLAM Base Object". Nothing in the visible stack is at fault.

## Root cause

`libbasalt.so` and `basalt_slam_node` are compiled with different `-march` settings, so they disagree on the memory layout of `basalt::Controller`. Constructing it overflows the heap by 48 bytes; the next allocation-heavy operation (the loader's `dlopen`) walks the damaged arena and dies.

Chain:

1. `slam/CMakeLists.txt:185` does `add_subdirectory(ext/basalt)`; `:187` declares `basalt_slam_node` in the **parent** scope. (Pre-fix these were `:159`/`:161`; the pin added at `:182` shifted them.)
2. `ext/basalt/CMakeLists.txt:85-86` defaults `CXX_MARCH` to `native`; `:90` forms `-march=native`; `:265` folds it into `CMAKE_CXX_FLAGS`. CMake `set()` without `PARENT_SCOPE` is directory-scoped, so this **never reaches the node's own TUs**.
3. `-march=native` on the build host (AMD Ryzen 9 9950X, Zen 5) enables AVX-512, which defines `__AVX512F/DQ/BW/VL/CD__`. Eigen reads those macros and raises `EIGEN_MAX_ALIGN_BYTES` from **16 to 64**, changing the alignment — and therefore `sizeof` and member offsets — of every fixed-size vectorizable Eigen type and everything containing one by value.
4. `basalt::Controller` holds `basalt::Calibration<double> calib_` by value (`ext/basalt/include/basalt/controller.h:71`), which holds `CalibGyroBias<double>` by value (`basalt-headers/.../calibration.hpp:131`), whose member is `Eigen::Matrix<double,12,1>` — 96 bytes, aligned 16 without AVX and 32 with it.
5. `src/basalt/slam.cpp:26` calls `std::make_unique<basalt::Controller>(...)`. `make_unique` is inlined into the **non-AVX** TU → `operator new(1008)`. `Controller::Controller()` lives in `ext/basalt/src/controller.cpp` inside **libbasalt.so** and initialises a **1056-byte** object → 48-byte overflow into glibc chunk metadata. The object is also only 16-byte aligned while the library assumes 32.

## Measured evidence (all reproducible from the existing build tree)

| Check | Command | Result |
|---|---|---|
| Flags differ | `grep ^CXX_FLAGS build/slam/ext/basalt/CMakeFiles/basalt.dir/flags.make` vs `build/slam/CMakeFiles/basalt_slam_node.dir/flags.make` | basalt: `… -DEIGEN_DONT_PARALLELIZE -march=native -O3 -g …`; node: `-Wall -O3 -std=c++17 -O2 -g -DNDEBUG` (no `-march`) |
| Reached codegen | `objdump -d build/slam/ext/basalt/libbasalt.so \| grep -c '%zmm'` | 40093 (`%ymm` 28494, `%k0-7` 328); node objects: **0** `ymm`, **0** `zmm` |
| Layout diverged | `objdump --dwarf=info <obj> \| grep -A6 'DW_AT_name.*: Controller$'` | `controller.cpp.o`: `byte_size 1056`, `alignment 32` — `src/basalt/slam.cpp.o`: `byte_size 1008`, `alignment 16` (`Calibration<double>`: 416 vs 384) |
| Mechanism isolated | 20-line `sizeof/alignof/offsetof` program against vendored Eigen, compiled with/without `-march=native` | `EIGEN_MAX_ALIGN_BYTES` 16 vs 64; model struct 224/align 16 vs 288/align 32 |

`EIGEN_MAX_ALIGN_BYTES` by flag on this toolchain (Eigen 3.4): none/`-msse2` → 16; `-mavx`, `-mavx2 -mfma`, `-mavx512f` alone → 32; `-march=native` (full AVX-512 set) → 64. Effective alignment of a type is `min(EIGEN_MAX_ALIGN_BYTES, largest power of two dividing sizeof(T))` — hence `Matrix<double,12,1>` (96 B) is 32-aligned even when the bound is 64, while `Matrix<double,9,1>` (72 B) is 8-aligned either way and is harmless.

## Gotchas worth remembering

- **`-march` is an ABI switch, not an optimisation flag, in any Eigen-based codebase.** Two TUs sharing a type must agree on it, exactly like `-std` or `_GLIBCXX_USE_CXX11_ABI`.
- **A crash inside `ld.so`, `malloc`, `free` or `operator new` almost never indicates a bug at that location.** Those are the detectors of heap corruption committed earlier.
- **`add_subdirectory` scoping hides this.** Any vendored subproject that edits `CMAKE_CXX_FLAGS` creates a silent ABI boundary with its parent's targets.
- **This is x86-only.** AArch64/NEON is 128-bit, so `EIGEN_MAX_ALIGN_BYTES` is 16 on the Jetson Orin target regardless of `-march` — the bug cannot reproduce there.
- **Same hazard, second boundary:** `Sophus::SE3d` (contains `Quaterniond`, 32 B) is 16-aligned in the node and 32-aligned in basalt; `src/basalt/slam.cpp:50` constructs one and passes it by reference into `Controller::initialize`. `SE3f` happens to be safe (16 B `Quaternionf`).
- **`scripts/build_ros_packages.sh:37`** builds with `RelWithDebInfo`. It **now** passes `-DCXX_MARCH=${CXX_MARCH}` (selected per `uname -m` at `:26-34`, overridable from the environment); before the fix it passed none, which is why the default `native` applied. It also explains the node's `-O2 -g -DNDEBUG` (CMake's RelWithDebInfo) versus basalt's `-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN` (basalt overrides the RelWithDebInfo flags in its own scope).
- **Keep `-g` in release builds.** The DWARF `DW_AT_byte_size`/`DW_AT_alignment` comparison is what made this provable in minutes; without it the diagnosis needs ASan.
- **Upstream basalt agrees `native` is dev-only:** `ext/basalt/.gitlab-ci.yml` uses `CXX_MARCH: native` for the dev build (line 6) but pins `haswell` for every packaged `.deb` job (lines 158, 170, 183).

## The fix, as shipped 2026-07-25 (see the plan for detail)

1. **Structural:** pin `EIGEN_MAX_ALIGN_BYTES=16` across both the `ext/basalt` subdirectory and the parent `slam` targets, via a single `add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16 NRT_EIGEN_ABI_PIN=16)` placed **before** `add_subdirectory(ext/basalt)`. 16 (not 32) because every other ROS 2 package in the process — `rclcpp`, `tf2_eigen`, `cv_bridge` — is distribution-built without `-march`, so 16 makes basalt agree with the whole process rather than just with its own executable. Eigen keeps vectorizing, using unaligned moves. The definition must travel via `CXX_DEFINES` rather than `CXX_FLAGS`, because `ext/basalt/CMakeLists.txt:255` overwrites `CMAKE_CXX_FLAGS` wholesale. `NRT_EIGEN_ABI_PIN` then backs a `static_assert` on **each** side of the boundary — `include/slam/slam.hpp` and `ext/basalt/src/controller.cpp` — since the two sides share no header; both are `#ifdef`-guarded so standalone upstream basalt builds are unaffected.
2. **Hygiene:** set `CXX_MARCH` explicitly and **per architecture** (`x86-64-v3` for amd64, an `armv8.2-a`-family value for the Orin) instead of inheriting `native`. A single global value cannot serve the `buildx --platform linux/amd64,linux/arm64` matrix — `ext/basalt/CMakeLists.txt:79` still emits `-march=` on Linux/aarch64, where an x86 value is a hard error.

**Where each piece landed:** the pin is `ros_ws/src/slam/CMakeLists.txt` in the `elseif(SLAM_TYPE STREQUAL "BASALT")` branch, immediately before `add_subdirectory(ext/basalt)`; the guards are in `ros_ws/src/slam/include/slam/slam.hpp` (after `#include <Eigen/Geometry>`) and `ros_ws/src/slam/ext/basalt/src/controller.cpp` (after the includes, before `namespace basalt`); the architecture selection is a `case "$(uname -m)"` block in `scripts/build_ros_packages.sh` that sets `CXX_MARCH` and passes `-DCXX_MARCH=…` through `--cmake-args`, overridable from the environment; and a warning comment sits above `ext/basalt/CMakeLists.txt`'s `if(NOT CXX_MARCH)` default. A matching NOTE in the `ORBSLAM3` branch records that the same pin is required there if `-march` is ever introduced.

**Not yet verified on hardware.** The evidence commands in the table above must be re-run after a clean rebuild (`rm -rf /ws/ros_ws/{build,install,log}/slam`) to confirm both sides now report the same `DW_AT_byte_size`/`DW_AT_alignment`; the container this was authored in has no ROS or toolchain.

## Related defects found in the same investigation

All are now fixed except where noted. Status as re-verified 2026-07-25 **after** implementation.

- `src/basalt/node.cpp:87-95` — the SLAM-mode selection formerly passed a hardcoded `"monocular-only"` regardless of `slam_type`. **Fixed manually 2026-07-25**: `VSLAM` → `"monocular-only"` → `SlamMode::VO`, `VISLAM` → `"monocular-inertial"` → `SlamMode::VIO`. Consistent end to end.
- `src/basalt/node.cpp:153` — a raw `std::cout << "Received Image"` fired once per frame, bypassing the ROS logger and un-rate-limited. **Fixed 2026-07-25** to `RCLCPP_DEBUG`.
- `src/basalt/slam.cpp:53-54` — was `basalt::OpticalFlowInput::Ptr input;` default-constructed null and immediately dereferenced. A manual fix introduced `std::make_shared<basalt::OpticalFlowInput()>`, which is **ill-formed** (the template argument names a function type — "function taking no arguments returning `OpticalFlowInput`" — and the call parentheses are missing) and was a hard build blocker for the whole package. **Fixed 2026-07-25** to `std::make_shared<basalt::OpticalFlowInput>()`. `OpticalFlowInput` is a plain aggregate (`int64_t t_ns`, `std::vector<ImageData> img_data`) with no user-declared constructor, so it takes no camera count and value-initialisation is correct.
- `src/basalt/node.cpp:105-107` — the VISLAM IMU subscription requested the `rclcpp` default (RELIABLE) while `/ap/imu/experimental/data` is published **BEST_EFFORT** by AP_DDS, so the endpoints never matched and the VIO estimator received no IMU data. Silent: `ros2 node info` still listed the subscription. **Fixed 2026-07-25** to `rclcpp::SensorDataQoS()`, an exact match for the offer. See [`ros-qos-compatibility.md`](ros-qos-compatibility.md).
- `src/basalt/node.cpp:97-98` — `std::make_unique<tf2_ros::TransformBroadcaster>(tf2_ros::TransformBroadcaster(this))` constructed and copied a needless temporary, creating and destroying a spare `/tf` publisher. **Fixed 2026-07-25** to `std::make_unique<tf2_ros::TransformBroadcaster>(this)`.
- `CMakeLists.txt:143-145` — `install(TARGETS DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")` had an empty target list and an unset variable, so it installed nothing to the absolute path `/slam`. Inert under the default `SLAM_TYPE=BASALT`; blocked `-DSLAM_TYPE=ORBSLAM3`. **Deleted 2026-07-25**; the Python side is installed by `ament_python_install_package()` and the C++ targets by the `install(TARGETS orbslam3_mono_node …)` block below it. Note `djinn start sitl orbslam3` remains broken for unrelated reasons — see [`/ws/context/djinn-workflows.md`](../../context/djinn-workflows.md).
- `launch/basalt_slam.launch.py:72` — an earlier revision of this file had a gdb prefix reading `ex run` instead of `-ex run`, which is why `/basalt_slam_node/change_state` never appeared and the controller waited forever. **No longer present**: the current working tree adds a syntactically correct `prefix='gdb -ex "set confirm off" -ex run -ex bt -ex q --args'`. Kept unconditional by decision; note a gdb-wrapped node does not forward `SIGTERM` normally, which is why `/ws/djinn:284` needs `kill -9`.

See also [`slam-package.md`](slam-package.md), [`basalt-node-cpp.md`](basalt-node-cpp.md), [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md), [`ros-qos-compatibility.md`](ros-qos-compatibility.md), and the package-scoped [`src/slam/context/`](../src/slam/context/README.md).
