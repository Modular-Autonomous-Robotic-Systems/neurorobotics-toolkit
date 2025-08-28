#ifndef VIDEO_LOGGING_DRIVER_HPP_
#define VIDEO_LOGGING_DRIVER_HPP_

#include "controllers/common/controller.h" 
#include "ardupilot_msgs/msg/status.hpp" 
#include "std_msgs/msg/string.hpp"
#include <string>
#include <memory>

const std::string DRIVER_DEFAULT_LIFECYCLE_NODE_TO_MANAGE = "video_logger_test";
const std::string DRIVER_DEFAULT_AP_STATUS_TOPIC = "/ap/status";
const std::string DRIVER_DEFAULT_AP_TYPE = "ardupilot";
const std::vector<std::string> DRIVER_SUPPORTED_AP_TYPES = {"ardupilot", "tello"};

class VideoLoggingDriver : public LifecycleControllerBase
{
	public:
		explicit VideoLoggingDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
		virtual ~VideoLoggingDriver();

	private:
		std::string mpLifecycleNodeNameToManage; 
		std::string mpAPStatusTopic;
		std::string mpAPType;

		bool mpPreviousArmedStatus = false;
		bool mpPreviousFlyingStatus = false;
		uint8_t mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
		int mpStatusMessageCounter = 0;

		rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr mpArduPilotStatusSubscriber;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpTelloStatusSubscriber;
		
		void InitializeDriverParameters(); 
		void SetupROSInterfaces();

		void ArduPilotStatusCallback(const ardupilot_msgs::msg::Status::ConstSharedPtr msg);
		void TelloStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg);
		
		void OnVideoLoggerChangeStateResponse(
			uint8_t attemptedTransitionId,
			bool success,
			lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response);

		void OnVideoLoggerGetStateResponse(
			const std::string& context,
			bool success,
			lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response);
		
		void AttemptDriverSpecificGracefulShutdown(); 

		std::mutex mpcVideoLoggerStateMutex;
		std::mutex mpcStatusMutex;
};

#endif // VIDEO_LOGGING_DRIVER_HPP_
