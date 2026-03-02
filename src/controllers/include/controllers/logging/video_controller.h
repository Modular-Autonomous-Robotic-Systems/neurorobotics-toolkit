#ifndef VIDEO_LOGGING_DRIVER_HPP_
#define VIDEO_LOGGING_DRIVER_HPP_

#include "ardupilot_msgs/msg/status.hpp"
#include "controllers/common/controller.h"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>

const std::string DRIVER_DEFAULT_LIFECYCLE_NODE_TO_MANAGE = "video_logger_test";
const std::string DRIVER_DEFAULT_AP_STATUS_TOPIC = "/ap/status";
const std::string DRIVER_DEFAULT_AP_TYPE = "ardupilot";
const std::vector<std::string> DRIVER_SUPPORTED_AP_TYPES = {"ardupilot",
                                                            "tello"};
const std::string START_SUBSCRIBER_TOPIC_DEFAULT =
    "/video_logging_driver/start";
const std::string STOP_SUBSCRIBER_TOPIC_DEFAULT = "/video_logging_driver/stop";

class VideoLoggingDriver : public LifecycleControllerBase {
  public:
    explicit VideoLoggingDriver(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~VideoLoggingDriver();

  private:
    std::string mpLifecycleNodeNameToManage;
    std::string mpAPStatusTopic;
    std::string mpAPType;
    std::string mpStartSubscriberTopic;
    std::string mpStopSubscriberTopic;

    bool mpPreviousArmedStatus = false;
    bool mpPreviousFlyingStatus = false;
    uint8_t mpVideoLoggerKnownState =
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    int mpStatusMessageCounter = 0;

    rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr
        mpArduPilotStatusSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        mpTelloStatusSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpStartSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpStopSubscriber;

    void InitializeDriverParameters();
    void SetupROSInterfaces();

    void ArduPilotStatusCallback(
        const ardupilot_msgs::msg::Status::ConstSharedPtr msg);
    void TelloStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    void StartCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    void StopCallback(const std_msgs::msg::String::ConstSharedPtr msg);

    void OnVideoLoggerChangeStateResponse(
        uint8_t attemptedTransitionId, bool success,
        lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response);

    void OnVideoLoggerGetStateResponse(
        const std::string &context, bool success,
        lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response);

    void AttemptDriverSpecificGracefulShutdown();

    std::mutex mpcVideoLoggerStateMutex;
    std::mutex mpcStatusMutex;
};

#endif // VIDEO_LOGGING_DRIVER_HPP_
