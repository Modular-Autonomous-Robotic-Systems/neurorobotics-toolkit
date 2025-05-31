#ifndef VIDEO_LOGGING_DRIVER_HPP_
#define VIDEO_LOGGING_DRIVER_HPP_

#include "controllers/common/controller.h" 
#include "ardupilot_msgs/msg/status.hpp" 
#include <string>
#include <memory>

const std::string DRIVER_DEFAULT_LIFECYCLE_NODE_TO_MANAGE = "video_logger_test";
const std::string DRIVER_DEFAULT_AP_STATUS_TOPIC = "/ap/status";

class VideoLoggingDriver : public LifecycleControllerBase
{
	public:
		explicit VideoLoggingDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
		virtual ~VideoLoggingDriver();

	private:
		std::string mpLifecycleNodeNameToManage; 
		std::string mpAPStatusTopic;

		bool mpPreviousArmedStatus = false;
		bool mpPreviousFlyingStatus = false;
		uint8_t mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
		int mpStatusMessageCounter = 0;

		rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr mpStatusSubscriber;
		
		void InitializeDriverParameters(); 
		void SetupROSInterfaces();

		void StatusCallback(const ardupilot_msgs::msg::Status::ConstSharedPtr msg);
		
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
