# BasaltSLAMNode anatomy (node.cpp + node.hpp)

Files: `src/slam/src/basalt/node.cpp`, `src/slam/include/slam/basalt/node.hpp` (submodule).

## Current header (`node.hpp`)
```cpp
class BasaltSLAMNode : public SlamNode {
 public:
  BasaltSLAMNode(); ~BasaltSLAMNode();
  CallbackReturn on_configure / on_activate / on_deactivate / on_cleanup (...) override;
 protected:
  std::string mpCameraTopicName;
  std::string mpCalibrationFilePath;
  std::string mpConfigurationFilePath;
  eSLAMType   mpSLAMType;
 private:
  Frame mpCurrentFrame;
  std::unique_ptr<BasaltSLAM> mpSlam = nullptr;
  Eigen::Matrix3d mpBasaltToROSTransform;
  cv_bridge::CvImagePtr m_cvImPtr;
  rclcpp::Subscription<ImageMsg>::SharedPtr mpFrameSubscriber;
  rclcpp::Publisher<ImageMsg>::SharedPtr mpAnnotatedFramePublisher;
  void Update(); void GrabImage(const ImageMsg::SharedPtr); void GrabIMU(const ImuMsg::SharedPtr);
  void PublishFrame();
};
```

## Current constructor `BasaltSLAMNode()` (node.cpp:5-32)
Declares string params (all via `rcl_interfaces::msg::ParameterDescriptor`, `.type = 4` = STRING):
`camera_topic_name`(default `/camera`), `calibration_file_path`(""), `configuration_file_path`(""),
`slam_type`(default `VSLAM`). Sets `mpBasaltToROSTransform = Identity`.
**Missing: `imu_topic_name` param.**

## `on_configure` (node.cpp:40-62)
Reads `mpCameraTopicName`, `mpCalibrationFilePath`, `mpConfigurationFilePath`; maps `slam_type`
string → `mpSLAMType` (`VSLAM`/`VISLAM`). Logs them. `return CallbackReturn::SUCCESS;`
**Missing: read `mpIMUTopicName`; missing VISLAM validation (fail if empty).**

## `on_activate` (node.cpp:83-128, current as of 2026-07-27)
Creates `mpSlam = make_unique<BasaltSLAM>(logger, mpConfigurationFilePath, mpCalibrationFilePath,
setupCameraType)` where `setupCameraType` follows `mpSLAMType`, then `mpTfBroadcaster`,
`mpFrameSubscriber`(→`GrabImage`, topic `mpCameraTopicName`, qos 10), `mpIMUSubscriber`(→`GrabIMU`,
`rclcpp::SensorDataQoS()`) when VISLAM, and `mpAnnotatedFramePublisher`(`~/annotated_frame`).
Ends with `return SlamNode::on_activate(previous_state);` — the base call is **last**, because the
publishers are created above it. See `slam-package.md` for why that ordering is load-bearing.

## `on_deactivate` (node.cpp:130-152, current as of 2026-07-27)
Opens with `CallbackReturn result = SlamNode::on_deactivate(previous_state);` — the base call is
**first**, while the publishers still exist — then resets (if set) `mpAnnotatedFramePublisher`,
`mpFrameSubscriber`, `mpIMUSubscriber`, `mpTfBroadcaster`, `mpSlam`, and returns `result`.

## Already-implemented `GrabIMU` (node.cpp:136-145)
```cpp
void BasaltSLAMNode::GrabIMU(const ImuMsg::SharedPtr msg) {
  if (mpSLAMType == eSLAMType::VISLAM) {
    std::shared_ptr<Imu> imuPtr = std::make_shared<Imu>(msg);
    mpSlam->GrabIMU(imuPtr);
  } else { /* drop */ }
}
```
So the callback exists; only the subscriber creating it is missing.

## What the 4 C++ tasks add (exact)
1. ctor: declare `imu_topic_name` (default `""`, descriptor type 4).
2. on_configure: `mpIMUTopicName = get_parameter("imu_topic_name").as_string();`
   then `if (mpSLAMType == eSLAMType::VISLAM && mpIMUTopicName.empty()) { RCLCPP_ERROR(...);
   return CallbackReturn::FAILURE; }`.
3. on_activate: `mpIMUSubscriber = this->create_subscription<ImuMsg>(mpIMUTopicName, 10,
   std::bind(&BasaltSLAMNode::GrabIMU, this, std::placeholders::_1));`  (guard on VISLAM).
4. on_deactivate: `if (mpIMUSubscriber) mpIMUSubscriber.reset();`.
Header additions: `std::string mpIMUTopicName;` (protected) and
`rclcpp::Subscription<ImuMsg>::SharedPtr mpIMUSubscriber;` (private).
