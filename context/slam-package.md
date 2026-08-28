# src/slam (C++ compute nodes) — class map

Submodule (`ros2_orb_slam3.git`, branch main). Editing here changes the submodule tree.

## Message typedefs & data classes — `src/slam/include/slam/slam.hpp`
```cpp
using ImageMsg = sensor_msgs::msg::Image;   // line 38
using ImuMsg   = sensor_msgs::msg::Imu;     // line 39  (already includes <sensor_msgs/msg/imu.hpp>)
enum class eSLAMAlgorithm { NOT_SET=-1, ORBSLAM3=0, BASALT=1 };  // line 120
enum class eSLAMType { VSLAM=0, VISLAM=1 };                       // line 121
class Imu   : public Data { Imu(ImuMsg::SharedPtr msg); ... toBasaltImuData(); };  // ~56
class Frame : public Data { Frame(shared_ptr<cv::Mat>, long&); getImage(); };     // ~105
class Slam { virtual bool TrackMonocular(...)=0; virtual bool TrackMonocularIMU(...); ... }; // ~138
```

## Base lifecycle node — `src/slam/include/slam/node.hpp`
```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class SlamNode : public rclcpp_lifecycle::LifecycleNode {
  SlamNode(std::string nodeName);
  Sophus::SE3f mpTwc;
 protected:
  void Update();
  std::unique_ptr<tf2_ros::TransformBroadcaster> mpTfBroadcaster;
  std::string mpSettingsFilePath;
};
```

## Basalt wrapper — `src/slam/include/slam/basalt/slam.h`, `src/slam/src/basalt/slam.cpp`
```cpp
class BasaltSLAM : public Slam {
  BasaltSLAM(logger, configFilePath, calibFilePath, setupCameraType);
  bool TrackMonocular(Frame&, Sophus::SE3f&) override;   // true only when tcw was written
  void GrabIMU(std::shared_ptr<Imu>& data);   // pushes data->toBasaltImuData() to mpController->GrabIMU
  ...
  basalt::SlamMode mpSlamMode;  // enum class SlamMode { VO, VIO }; (controller.h:16)
};
```
- `InitialiseSlam`: `setupCameraType == "monocular-only"` → `mpSlamMode = VO`. **No VIO branch yet**
  → even in VISLAM the Basalt controller is built in VO mode. IMU is still queued via GrabIMU, but
  whether VO mode consumes it internally is a KNOWN RISK (see PLAN "Known limitations"). Out of scope
  for the enumerated tasks (those only wire the ROS2 subscriber + lifecycle).

## Basalt compute node — see `context/basalt-node-cpp.md`.
Node name: **`basalt_slam_node`** (from `SlamNode("basalt_slam_node")`).
Lifecycle services therefore: `/basalt_slam_node/change_state`, `/basalt_slam_node/get_state`.

## Managed publishers are now activated, and the reason it never mattered before (2026-07-27)

`SlamNode` derives from `rclcpp_lifecycle::LifecycleNode`, so `this->create_publisher(...)` inside `on_activate` returns an `rclcpp_lifecycle::LifecyclePublisher<T>` and registers it as a managed entity. Until 2026-07-27 neither `BasaltSLAMNode` nor `VisualSlamNode` called the base implementation from `on_activate` or `on_deactivate`, so those managed publishers were never activated.

Frames nevertheless flowed, and no "publisher is not activated" warning ever appeared. The reason is that `rclcpp::Publisher<T>::publish` is **not virtual** and `LifecyclePublisher<T>::publish` shadows it rather than overriding it, while every publisher member in this package is declared as `rclcpp::Publisher<T>::SharedPtr` (`include/slam/basalt/node.hpp:39`, `include/slam/orbslam3/monocular.hpp:66-69`). Each `publish()` call therefore binds statically to the base implementation and bypasses the activation check. The defect was latent rather than active, and it would have become live the moment a member's declared type was changed to `LifecyclePublisher<T>::SharedPtr`, which is the natural thing to do, at which point publishing would stop with only a throttled log line to show for it.

Both classes now delegate. `BasaltSLAMNode::on_activate` (`src/basalt/node.cpp`) ends with `return SlamNode::on_activate(previous_state);` and `on_deactivate` opens with `CallbackReturn result = SlamNode::on_deactivate(previous_state);` and returns `result`. `VisualSlamNode` (`src/orbslam3/monocular.cpp`) does the same for its two publishers.

Two ordering rules, and they are opposite ways round, which is the whole trap.

The base call belongs at the **end** of `on_activate`, not the top, because the publishers are created inside that callback and `LifecycleNode::on_activate` only activates the managed entities that exist when it is called. Calling the base first would activate an empty set and silently do nothing. Conversely it belongs at the **start** of `on_deactivate`, while the publishers still exist, so they are deactivated rather than destroyed while still marked active. The commented-out `mpMapPublisher` in `monocular.cpp` carries a note to this effect, since enabling it below the base call would silently leave it unactivated.

`SlamNode` declares no `on_activate` or `on_deactivate` of its own (`include/slam/node.hpp`), so `SlamNode::on_activate` resolves to `rclcpp_lifecycle::LifecycleNode::on_activate`. Naming the immediate base rather than `LifecycleNode` keeps the call correct should `SlamNode` ever gain an override. The base returns `CallbackReturn::SUCCESS` unconditionally in Humble, so propagating its return value introduces no new failure path.

`tf2_ros::TransformBroadcaster` is unaffected either way, because it creates a plain publisher through the node interfaces rather than a managed one, which is why the TF path always looked healthy.

The member-type correction, declaring these members as `LifecyclePublisher<T>::SharedPtr`, is deliberately still deferred, so the activation change can be validated on its own first. See [`/ws/plans/slam-lifecycle-ap-status-reconciler.md`](../../plans/slam-lifecycle-ap-status-reconciler.md) §7 and §8.

## Tracking return contract

Since 2026-08-23 every `TrackMonocular` and `TrackMonocularIMU` in the hierarchy returns `bool` rather than `void`, `true` meaning the backend wrote a pose into the `tcw` out parameter and `false` meaning the frame was dropped or tracking failed, in which case `tcw` is untouched and must not be published. `BasaltSLAMNode::GrabImage` gates `mpTwc = tcw.inverse()` and `Update()` on this flag. `MonoORBSLAM3::TrackMonocular` derives the flag from `mpORBSlam3->GetTrackingState() == ORB_SLAM3::Tracking::OK`, so `RECENTLY_LOST` and the pre initialisation states report `false` even though ORB-SLAM3 returns an identity pose for them. `MonoMORBSLAM::TrackMonocular` reports whether `MORB_SLAM::MonoPacket::pose`, an `std::optional`, held a value, and writes `tcw` only in that case, which also removes an unconditional dereference of an empty optional. The base `Slam::TrackMonocularIMU` returns `false` after logging. `VisualSlamNode::GrabImage` (`src/slam/src/orbslam3/monocular.cpp:152`) still ignores the flag and publishes unconditionally, unlike `BasaltSLAMNode::GrabImage`, so the ORB-SLAM3 path can still broadcast a pose for an untracked frame. (Correction, 2026-08-23. The earlier `void` signature is recorded above as changed because it gave the caller no way to distinguish a tracked frame from a dropped one.)
