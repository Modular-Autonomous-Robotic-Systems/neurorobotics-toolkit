# src/controllers (lifecycle controllers, Python and C++)

Regular (non-submodule) package. `ament_cmake` plus `ament_cmake_python`, so it ships both a Python package and C++ executables.

## Layout

```
src/controllers/
  controllers/                       # python pkg installed via ament_python_install_package
    __init__.py
    slam/
      __init__.py
      driver_node.py                 # class SLAMDriverNode(Node), the shared base
      mono_driver_node.py            # class MonoDriver(SLAMDriverNode) + main()
      basalt_driver_node.py          # class BasaltSLAMDriver(SLAMDriverNode) + main()
  scripts/
    orbslam3_monocular_controller    # executable entry-point script
    basalt_slam_controller           # executable entry-point script
  include/controllers/
    common/controller.h              # class LifecycleControllerBase
    logging/video_controller.h       # class VideoLoggingDriver
    tello/controller.h               # class TelloController
  src/
    common/controller.cpp
    logging/video_controller.cpp     # manages managed_logger_node
    tello/controller.cpp
  launch/                            # *.launch.py, installed to share/controllers/launch
  CMakeLists.txt  package.xml
```

## Entry-point pattern

```python
#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from controllers.slam.basalt_driver_node import BasaltSLAMDriver


def main():
    driver = None
    executor = None
    try:
        rclpy.init()
        driver = BasaltSLAMDriver()
        executor = MultiThreadedExecutor()
        executor.add_node(driver)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if driver is not None:
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

Registered in `CMakeLists.txt` as `install(PROGRAMS scripts/<name> DESTINATION lib/${PROJECT_NAME})` and launched as `package="controllers" executable="<name>"`.

> A new SLAM controller needs the same triple, a script in `scripts/`, an `install(PROGRAMS ...)` line, and a `launch_ros.actions.Node(executable=...)`.

The `MultiThreadedExecutor` is not optional as of 2026-07-27. `SLAMDriverNode` puts its lifecycle observation path, that is the `transition_event` subscription, the service future done-callbacks and the service-readiness timer, in a `ReentrantCallbackGroup`, and leaves the status subscription in the node's default mutually exclusive group. Under `rclpy.spin` that separation is inert. The module-level `main()` in `mono_driver_node.py` and `basalt_driver_node.py` now mirrors the script exactly. The `MultiThreadedExecutor` import that `mono_driver_node.py` used without importing, a latent `NameError`, was fixed at the same time.

## `SLAMDriverNode` (`controllers/slam/driver_node.py`)

The shared base for both SLAM back ends. It is a manager, not a lifecycle node. See [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md) for the control flow and the reconciler pattern it implements.

Parameters declared and read in the constructor, `slam_compute_node_name` (default `orbslam3_mono_node`), `ardupilot_status_topic` (`/ap/status`), `tello_status_topic` (`/tello_state`), `drone_type` (`ardupilot`).

Module-level tables above the class, which are the reason the driver logs names rather than integers.

| Name | Purpose |
|---|---|
| `STATE_NAMES`, `TRANSITION_NAMES` | id to label, built from the `lifecycle_msgs` constants so they cannot drift from the distribution |
| `PRIMARY_STATES` | the four ids that may overwrite the state mirror; transition states are logged only |
| `TRANSITION_STEP` | `(known, desired)` to the single transition that steps toward it |
| `TRANSITION_RESULT_STATE` | the state a confirmed transition leaves the node in, the same-instant fallback for the mirror |
| `TRANSITION_EVENT_LOGS` | 27 entries keyed on `msg.transition.id`, one distinct operator-facing message each |
| `RECONCILE_AFTER_TRANSITIONS` | the five success outcomes that may re-enter the reconciler |
| `COPTER_MODE_NAMES`, `_mode_name` | diagnostic only, guarded on `Status.APM_ARDUCOPTER` because mode numbers collide across vehicle types |
| `TRANSITION_TIMEOUT_SEC`, `SERVICE_POLL_PERIOD_SEC` | 10.0 and 1.0 |

Methods, in file order. `_await_lifecycle_services`, `_query_state`, `_on_get_state_response`, `_request_transition`, `_on_change_state_response`, `_tello_status_callback`, `_ap_status_callback`, `_desired_state`, `_phase_description`, `_reconcile`, `_transition_event_callback`, and the two deprecated synchronous helpers `_get_state` and `_change_state`, retained for out-of-tree callers and called by nothing in this workspace.

Subclasses only call `super().__init__(node_name)` and then declare their own algorithm-specific parameters and helpers (CvBridge, frame counters). Neither overrides any lifecycle method, so a change to the base reaches both back ends identically.

## C++ controllers

`LifecycleControllerBase` (`common/controller.{h,cpp}`) supplies the client registry, `AsyncCallChangeState`, `AsyncGetNodeState`, `SyncCallChangeState`, the worker-thread pool behind `EnqueueServiceResponseHandlerTask`, and the `GetStateLabel`/`GetTransitionLabel` helpers. Both `VideoLoggingDriver` and `TelloController` derive from it, so any edit to it reaches both.

`VideoLoggingDriver` (`logging/video_controller.{h,cpp}`) manages `managed_logger_node` and, since 2026-07-27, implements the same target-state reconciler as the Python driver. Its `main()` already used a `MultiThreadedExecutor`. See [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md).

Two standing items in the base class, both recorded in the plans rather than fixed. `WaitForAllRegisteredServices()` blocks in `VideoLoggingDriver`'s constructor, which violates the no-blocking-constructor rule. And `LifecycleControllerBase` inherits both `rclcpp::Node` and `std::enable_shared_from_this<LifecycleControllerBase>`, so any `shared_from_this()` would be ambiguous; there is no call site today.
