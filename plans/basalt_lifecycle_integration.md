# PLAN — Basalt Visual-Inertial SLAM lifecycle integration

Goal: extend the existing ORB_SLAM3-only Visual-SLAM lifecycle controller and the Basalt
compute node so **Visual-Inertial SLAM (VISLAM)** works end-to-end in SITL, by
(a) adding IMU wiring to the C++ `BasaltSLAMNode`, and (b) refactoring the Python driver into a
shared parent `SLAMDriverNode` with `MonoDriver` and a new `BasaltSLAMDriver` child, then
(c) wiring launch files + djinn so the controller actually drives the Basalt node.

> Read `context/README.md` first — it indexes all file/line references used below. `/ws` is the
> project root; run everything from `/ws/ros_ws` unless noted. `src/slam` is a **git submodule**.

Legend: **[TASK n]** = explicitly requested task. **[REQ]** = not enumerated but REQUIRED for the
controller to actually run (do not skip). **[NOTE]** = rationale / risk.

---

## Part A — C++ Basalt compute node (`src/slam`, submodule)

Files: `src/slam/include/slam/basalt/node.hpp`, `src/slam/src/basalt/node.cpp`.
`ImuMsg = sensor_msgs::msg::Imu` and `eSLAMType` are already visible via `slam/slam.hpp`.
`GrabIMU(const ImuMsg::SharedPtr)` already exists and already guards on `mpSLAMType == VISLAM`.

### A1 [REQ] Header — add the two missing members (`node.hpp`)
Add `mpIMUTopicName` next to the other string members (protected block), and `mpIMUSubscriber`
next to `mpFrameSubscriber` (private block).

```cpp
 protected:
    std::string mpCameraTopicName;
    std::string mpCalibrationFilePath;
    std::string mpConfigurationFilePath;
    std::string mpIMUTopicName;            // <-- ADD
    eSLAMType mpSLAMType;
 private:
    ...
    rclcpp::Subscription<ImageMsg>::SharedPtr mpFrameSubscriber;
    rclcpp::Subscription<ImuMsg>::SharedPtr mpIMUSubscriber;   // <-- ADD
    rclcpp::Publisher<ImageMsg>::SharedPtr mpAnnotatedFramePublisher;
```

### A2 [TASK 1] Declare `imu_topic_name` param in the constructor (`node.cpp`, `BasaltSLAMNode::BasaltSLAMNode`)
Mirror the existing string-parameter pattern; **default MUST be empty string** (Task 2 relies on it).
Insert after the `slam_type` param declaration, before `mpBasaltToROSTransform = ...`:

```cpp
    rcl_interfaces::msg::ParameterDescriptor imu_topic_name_desc;
    imu_topic_name_desc.description = "IMU Topic Name (required when slam_type == VISLAM)";
    imu_topic_name_desc.type = 4;  // PARAMETER_STRING
    this->declare_parameter<std::string>("imu_topic_name", "", imu_topic_name_desc);
```

### A3 [TASK 2] Read + validate in `on_configure` (`node.cpp`)
After the existing `mpConfigurationFilePath = ...` reads and the `slam_type`→`mpSLAMType` mapping,
add the read and the VISLAM validation. Return `FAILURE` if VISLAM and topic empty.

```cpp
    mpIMUTopicName = this->get_parameter("imu_topic_name").as_string();

    if (mpSLAMType == eSLAMType::VISLAM && mpIMUTopicName.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "slam_type is VISLAM but 'imu_topic_name' is empty. "
                     "Configuration failed.");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "IMU Topic Name: %s", mpIMUTopicName.c_str());
```
Keep the existing `return CallbackReturn::SUCCESS;` at the end unchanged.

### A4 [TASK 3] Create the IMU subscriber in `on_activate` (`node.cpp`)
After `mpFrameSubscriber` is created (and before/after the publisher — order irrelevant), create the
IMU subscriber **only for VISLAM** (VSLAM has no IMU topic and default is empty):

```cpp
    if (mpSLAMType == eSLAMType::VISLAM) {
        mpIMUSubscriber = this->create_subscription<ImuMsg>(
            mpIMUTopicName, 10,
            std::bind(&BasaltSLAMNode::GrabIMU, this, std::placeholders::_1));
    }
```
[NOTE] `GrabIMU` already drops data when not VISLAM, so guarding subscriber creation is belt-and-suspenders
and avoids subscribing to an empty topic name.

### A5 [TASK 4] Reset the subscriber in `on_deactivate` (`node.cpp`)
Add alongside the other resets (place it consistently, e.g. right after the `mpFrameSubscriber` reset):

```cpp
    if (mpIMUSubscriber) {
        mpIMUSubscriber.reset();
    }
```

[NOTE / KNOWN RISK — do NOT change unless asked] `on_activate` builds `BasaltSLAM(..., "monocular-only")`
which sets Basalt `SlamMode::VO`, not `VIO` (`slam.cpp: InitialiseSlam`). IMU is still queued via
`GrabIMU`, but full VIO fusion may require a `"monocular-inertial"` → `VIO` branch. This is outside the
enumerated tasks; record it as a follow-up if VISLAM tracking underperforms.

---

## Part B — Python driver refactor (`src/controllers/controllers/slam/`)

Create the parent, slim down `MonoDriver`, add `BasaltSLAMDriver`. Preserve backward compatibility:
after the split, `MonoDriver` must still declare/read **exactly the same params** as before (they are
now split across parent+child, with **no overlaps** → no double-declaration error).

### B1 [TASK 5,6,10] Create `src/controllers/controllers/slam/driver_node.py`
Move the four methods verbatim from `MonoDriver` and move the common imports here. Contents:

```python
#!/usr/bin/env python3
"""Base SLAM driver node: platform/autopilot lifecycle management shared by all SLAM algorithms."""

import rclpy
from ardupilot_msgs.msg import Status
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import String


class SLAMDriverNode(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        self.declare_parameter("slam_compute_node_name", "orbslam3_mono_node")
        self.declare_parameter("ardupilot_status_topic", "/ap/status")
        self.declare_parameter("tello_status_topic", "/tello_state")
        drone_type_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Type of drone supported by the SLAM driver. Can be one of- 'ardupilot' or 'tello'",
        )
        self.declare_parameter("drone_type", "ardupilot", drone_type_descriptor)

        self.drone_type = str(self.get_parameter("drone_type").value)
        self.slam_compute_node_name = str(self.get_parameter("slam_compute_node_name").value)
        self.ardupilot_status_topic = str(self.get_parameter("ardupilot_status_topic").value)
        self.tello_status_topic = str(self.get_parameter("tello_status_topic").value)

        self.get_logger().info(f"-------------- Received Autopilot and SLAM parameters --------------------------")
        self.get_logger().info(f"slam_compute_node_name: {self.slam_compute_node_name}")
        self.get_logger().info(f"ardupilot_status_topic: {self.ardupilot_status_topic}")
        self.get_logger().info(f"tello_status_topic: {self.tello_status_topic}")
        self.get_logger().info(f"drone_type: {self.drone_type}")
        self.get_logger().info(f"-------------------------------------------------------------")

        self.node_name = node_name

        self.get_logger().info(f"'{self.node_name}' initialized.")
        self.get_logger().info(f"Attempting handshake with lifecycle node '{self.slam_compute_node_name}'...")

        # --- Lifecycle Management ---
        self.service_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        change_state_service_name = f"/{self.slam_compute_node_name}/change_state"
        get_state_service_name = f"/{self.slam_compute_node_name}/get_state"
        self.change_state_client = self.create_client(
            ChangeState, change_state_service_name, callback_group=self.service_callback_group)
        self.get_state_client = self.create_client(
            GetState, get_state_service_name, callback_group=self.service_callback_group)

        while not self.change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"Lifecycle service '{change_state_service_name}' not available, waiting...")
        self.get_logger().info(f"Lifecycle service '{change_state_service_name}' now available")
        while not self.get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"Lifecycle service '{get_state_service_name}' not available, waiting...")
        self.get_logger().info(f"Lifecycle service '{get_state_service_name}' now available")
        current_state = self._get_state()
        self.get_logger().info(f"Current state of node {self.slam_compute_node_name}: {current_state}")

        # --- Autopilot Status Subscriber ---
        status_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_topic = ""
        if self.drone_type == "ardupilot":
            status_topic = self.ardupilot_status_topic
            self.status_subscriber = self.create_subscription(
                Status, status_topic, self._ap_status_callback, status_qos_profile)
        elif self.drone_type == "tello":
            status_topic = self.tello_status_topic
            self.status_subscriber = self.create_subscription(
                String, status_topic, self._tello_status_callback, status_qos_profile)
        self.get_logger().info(
            f"Subscribed to '{status_topic}'. Waiting for messages to control SLAM node...")

    # ----- MOVE these four methods VERBATIM from MonoDriver (mono_driver_node.py:206-342) -----
    def _tello_status_callback(self, msg: String): ...   # copy body unchanged
    def _ap_status_callback(self, msg: Status): ...      # copy body unchanged
    def _get_state(self, timeout_sec=5.0) -> int: ...    # copy body unchanged
    def _change_state(self, transition_id: int) -> bool: ...  # copy body unchanged
```
Copy the four method bodies **exactly** as they are in `mono_driver_node.py` today (see
`context/controllers-package.md` for their summaries; do not alter logic).
[NOTE] `driver_node.py` has **no `main()`** (Task 8) — it is a base class only.

### B2 [TASK 7,8] Rewrite `src/controllers/controllers/slam/mono_driver_node.py`
- Change imports: remove the ones moved to `driver_node.py`; keep only what `MonoDriver.__init__`
  + `main()` still use. Add `from controllers.slam.driver_node import SLAMDriverNode`.
- Change class to `class MonoDriver(SLAMDriverNode):` and replace its constructor with the Task-7
  body (declares only `settings_file_path`, `camera_topic_name`, `vocab_file_path`; reads them;
  logs; CvBridge; frame/work vars). `super().__init__(node_name)` runs the parent ctor first.
- **Delete** the four moved methods from `MonoDriver`.
- **Keep `main()` byte-for-byte identical** (Task 8).

Resulting top of file:
```python
#!/usr/bin/env python3
"""Python node for configuring and driving the MonocularMode (ORB_SLAM3) cpp node."""

import rclpy
from cv_bridge import CvBridge

from controllers.slam.driver_node import SLAMDriverNode


class MonoDriver(SLAMDriverNode):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)
        self.declare_parameter("settings_file_path", "/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml")
        self.declare_parameter("camera_topic_name", "/camera")
        self.declare_parameter("vocab_file_path", "/ws/config/slam/orbslam3/vocab.txt")
        self.settings_path = str(self.get_parameter("settings_file_path").value)
        self.camera_topic = str(self.get_parameter("camera_topic_name").value)
        self.vocab_file_path = str(self.get_parameter("vocab_file_path").value)
        self.get_logger().info(f"-------------- Received SLAM parameters --------------------------")
        self.get_logger().info(f"settings_path: {self.settings_path}")
        self.get_logger().info(f"camera_topic: {self.camera_topic}")
        self.get_logger().info(f"vocab_file_path: {self.vocab_file_path}")
        self.get_logger().info(f"-------------------------------------------------------------")
        self.node_name = node_name
        self.br = CvBridge()
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgs = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []


# main() below — UNCHANGED from current file (keep MultiThreadedExecutor usage as-is).
```
[NOTE] `main()` references `MultiThreadedExecutor` which the module never imported — this is a
pre-existing latent bug and the real entry-point (`scripts/orbslam3_monocular_controller`) uses its
own `rclpy.spin`, so `main()` is dead code. Task 8 says keep it unchanged → **leave it exactly as-is**
(do not "fix" the import, to honor the instruction and avoid behavior change).

### B3 [TASK 9] Create `src/controllers/controllers/slam/basalt_driver_node.py`
[NOTE] Task text says the path is `src/controllers/controller/slam/...` — that is a typo; the real
package dir is `controllers/controllers/slam/` (see `context/controllers-package.md`). Use the real path.

```python
#!/usr/bin/env python3
"""Python node for configuring and driving the Basalt SLAM cpp node (VSLAM / VISLAM)."""

import rclpy
from cv_bridge import CvBridge

from controllers.slam.driver_node import SLAMDriverNode


class BasaltSLAMDriver(SLAMDriverNode):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)
        self.declare_parameter("config_file_path", "/ws/ros_ws/src/slam/ext/basalt/data/sitl_config.json")
        self.declare_parameter("camera_topic_name", "/camera")
        self.declare_parameter("imu_topic_name", "/imu")
        self.declare_parameter("calib_file_path", "/ws/src/slam/ext/basalt/data/sitl_calib.json")
        self.config_path = str(self.get_parameter("config_file_path").value)
        self.camera_topic = str(self.get_parameter("camera_topic_name").value)
        self.imu_topic = str(self.get_parameter("imu_topic_name").value)
        self.calib_file_path = str(self.get_parameter("calib_file_path").value)
        self.get_logger().info(f"-------------- Received SLAM parameters --------------------------")
        self.get_logger().info(f"config_path: {self.config_path}")
        self.get_logger().info(f"camera_topic: {self.camera_topic}")
        self.get_logger().info(f"calib_file_path: {self.calib_file_path}")
        self.get_logger().info(f"-------------------------------------------------------------")
        self.node_name = node_name
        self.br = CvBridge()
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgs = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []


def main(args=None):
    try:
        rclpy.init(args=args)
        basalt_driver = BasaltSLAMDriver()
        rclpy.spin(basalt_driver)
        basalt_driver.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```
[NOTE] These controller-local params (`config_file_path`, `calib_file_path`, `imu_topic_name`,
`camera_topic_name`) are for logging/introspection only — the **compute node** gets its paths from
`slam/launch/basalt_slam.launch.py`, not from this controller. Their defaults may be stale (e.g.
`sitl_config.json` doesn't exist) — harmless. Add a `main()` here (mirrors `MonoDriver`) so a script
can import it.

---

## Part C — Entry point + launch wiring (make it actually run) [REQ]

The enumerated tasks add code but nothing yet *starts* a Basalt controller. Add these:

### C1 [REQ] Entry-point script `src/controllers/scripts/basalt_slam_controller`
Mirror `orbslam3_monocular_controller`:
```python
#!/usr/bin/env python3
from controllers.slam.basalt_driver_node import BasaltSLAMDriver
import rclpy
def main():
    try:
        rclpy.init(); basalt_driver = BasaltSLAMDriver(); rclpy.spin(basalt_driver)
        basalt_driver.destroy_node(); rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()
if __name__ == '__main__':
    main()
```
`chmod +x` it (git records the mode).

### C2 [REQ] Register it in `src/controllers/CMakeLists.txt`
Extend the existing `install(PROGRAMS ...)`:
```cmake
install(PROGRAMS
    scripts/orbslam3_monocular_controller
    scripts/basalt_slam_controller
    DESTINATION lib/${PROJECT_NAME}
)
```

### C3 [TASK: launch fix] Add `imu-topic` to `src/slam/launch/basalt_slam.launch.py`
Add the arg + wire it into the node params:
```python
    imu_topic_default = ""
    imu_topic = launch.substitutions.LaunchConfiguration("imu-topic", default=imu_topic_default)
    imu_topic_arg = launch.actions.DeclareLaunchArgument(
        "imu-topic", default_value=[imu_topic_default], description="IMU Topic name (required for VISLAM)")
    # ... in basalt_slam_node parameters dict, add:
    #     "imu_topic_name": imu_topic,
    # ... add imu_topic_arg to the returned nodes list (before basalt_slam_node).
```

### C4 [REQ] Add the controller node + forward args in `src/controllers/launch/basalt_slam_test.launch.py`
Currently it only includes `sitl.launch.py` + `basalt_slam.launch.py`. Add:
(a) declare/read the launch args it needs (`log-level`, `camera-topic`, `imu-topic`,
`calibration-file-path`, `configuration-file-path`, `slam-type`, `drone-type` default `ardupilot`,
`ap_status_topic_name` default `/ap/status`) — the calib/config/slam-type/camera/imu ones are the
same names already declared by `basalt_slam.launch.py`, so re-declaring them here (or reading them as
`LaunchConfiguration`) lets both the include-forwarding in (b) and the controller node in (c) share them;
(b) forward `camera-topic`, `calibration-file-path`, `configuration-file-path`, `slam-type`,
`imu-topic` to the `basalt_slam.launch.py` include via `launch_arguments`;
(c) add the Basalt controller node, **with `slam_compute_node_name` set to `basalt_slam_node`**:
```python
    basalt_controller_node = launch_ros.actions.Node(
        package="controllers",
        executable="basalt_slam_controller",
        name="basalt_controller",
        namespace="/",
        output="screen",
        respawn=False,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "slam_compute_node_name": "basalt_slam_node",
            "drone_type": drone_type,                       # LaunchConfiguration
            "ardupilot_status_topic": ap_status_topic_name, # LaunchConfiguration
            "camera_topic_name": camera_topic,
            "imu_topic_name": imu_topic,
            "config_file_path": configuration_file_path,
            "calib_file_path": calibration_file_path,
        }],
    )
```
Add `basalt_controller_node` into the timed actions **after** `basalt_slam_launch` so the compute
node's lifecycle services exist before the controller blocks on `wait_for_service`. (A `TimerAction`
delay like the ORB_SLAM3 launch's 3.0s is the safe pattern.) Keep declared `*_arg` objects in the
returned `LaunchDescription`.

### C5 [TASK 11] Update `djinn` (`/ws/djinn`, `djinn start sitl basalt`, ~lines 257-269)
Add the IMU topic (SITL IMU = `/ap/imu/experimental/data`). Insert into the `ros2 launch` args:
```
    slam-type:=VISLAM \
    imu-topic:=/ap/imu/experimental/data \
```
Keep `calibration-file-path:=.../sitl_calib.json`, `configuration-file-path:=.../sitl_config_vo.json`
as they are. Also update the sibling `src/basalt_slam_test.sh` for parity (add `imu-topic:=""` or the
SITL topic) so the two invocations stay consistent.

---

## Part D — Validation (run after implementing)

1. **C++ builds** (submodule): from `/ws/ros_ws`
   ```
   colcon build --packages-select slam
   ```
   Expect `basalt_slam_node` to compile — confirms `mpIMUTopicName` / `mpIMUSubscriber` members and
   the `ImuMsg` subscriber template resolve.

2. **Python controllers build/install**:
   ```
   colcon build --packages-select controllers
   ```
   Then confirm both scripts installed:
   ```
   ls install/controllers/lib/controllers/ | grep -E 'orbslam3_monocular_controller|basalt_slam_controller'
   ```

3. **Import / syntax check** (no double-declared params, imports resolve):
   ```
   source install/setup.bash
   python3 -c "from controllers.slam.driver_node import SLAMDriverNode; \
               from controllers.slam.mono_driver_node import MonoDriver; \
               from controllers.slam.basalt_driver_node import BasaltSLAMDriver; print('ok')"
   ```

4. **Backward compat — ORB_SLAM3 path unchanged**: launch `slam_controller.launch.py` (or
   `tello_orbslam3_active_mono_launch.py`) and confirm `orbslam3_controller` still handshakes with
   `orbslam3_mono_node` exactly as before (same params accepted, same log lines).

5. **VISLAM validation gate**: start `basalt_slam_node` alone, `configure` it with `slam_type:=VISLAM`
   and `imu_topic_name:=""` → `on_configure` must FAIL with the RCLCPP_ERROR. With a non-empty
   `imu_topic_name` → configure SUCCESS; after `activate`, `ros2 topic info <imu_topic>` shows the new
   subscriber; after `deactivate`, the subscriber is gone.

6. **End-to-end (SITL)**: `djinn start sitl basalt`. On arm, `basalt_controller` should drive
   `basalt_slam_node` UNCONFIGURED→INACTIVE→ACTIVE; `ros2 lifecycle get /basalt_slam_node` confirms.
   Verify Basalt receives IMU: `ros2 topic hz /ap/imu/experimental/data` and DEBUG log "received IMU msg".

---

## Checklist (all must be done)
- [ ] A1 header: `mpIMUTopicName`, `mpIMUSubscriber` members
- [ ] A2 (Task 1) ctor declares `imu_topic_name` default `""`
- [ ] A3 (Task 2) on_configure reads + VISLAM-empty → `FAILURE`
- [ ] A4 (Task 3) on_activate creates `mpIMUSubscriber` → `GrabIMU` (VISLAM)
- [ ] A5 (Task 4) on_deactivate resets `mpIMUSubscriber`
- [ ] B1 (Task 5,6,10) `driver_node.py` + `SLAMDriverNode` + 4 methods moved + common imports
- [ ] B2 (Task 7,8) `MonoDriver(SLAMDriverNode)` slimmed; `main()` unchanged
- [ ] B3 (Task 9) `basalt_driver_node.py` + `BasaltSLAMDriver`
- [ ] C1/C2 (REQ) `basalt_slam_controller` script + CMake install
- [ ] C3 `basalt_slam.launch.py` gains `imu-topic` → `imu_topic_name`
- [ ] C4 (REQ) `basalt_slam_test.launch.py` launches `basalt_controller` (slam_compute_node_name=basalt_slam_node) + forwards args
- [ ] C5 (Task 11) `djinn` + `basalt_slam_test.sh` add `imu-topic`
- [ ] D validation steps 1-6 pass

## Known limitations / follow-ups (not in scope)
- `BasaltSLAM` is constructed `"monocular-only"` → Basalt `SlamMode::VO`, even for VISLAM. If VISLAM
  tracking underperforms, add a `"monocular-inertial"` → `VIO` branch in `slam.cpp::InitialiseSlam`
  and select it in `node.cpp::on_activate` based on `mpSLAMType`.
- IMU/frame stream time-sync is assumed; desync degrades tracking silently (see
  `context/slam-lifecycle-flow.md`).
