#include "controllers/logging/video_controller.h"

using namespace std::chrono_literals;

VideoLoggingDriver::VideoLoggingDriver(const rclcpp::NodeOptions & options)
	: LifecycleControllerBase("video_logging_driver", options) 
{
	RCLCPP_INFO(this->get_logger(), "VideoLoggingDriver derived constructor starting...");

	this->InitializeDriverParameters();

	LifecycleControllerBase::ChangeStateCallbackType changeCb =
		std::bind(&VideoLoggingDriver::OnVideoLoggerChangeStateResponse, this,
				std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	LifecycleControllerBase::GetStateCallbackType getCb =
		std::bind(&VideoLoggingDriver::OnVideoLoggerGetStateResponse, this,
				std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	if (!this->RegisterNode(this->mpLifecycleNodeNameToManage, changeCb, getCb)) {
		RCLCPP_FATAL(this->get_logger(), "Failed to register node '%s' with base controller. Exiting.", this->mpLifecycleNodeNameToManage.c_str());
		rclcpp::shutdown();
		return;
	}

	this->WaitForAllRegisteredServices();

	RCLCPP_INFO(this->get_logger(), "Synchronously getting initial state for '%s'...", this->mpLifecycleNodeNameToManage.c_str());
	std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr> futureState =
		this->AsyncGetNodeState(this->mpLifecycleNodeNameToManage);

	std::future_status status = futureState.wait_for(this->mpServiceCallTimeoutMs);

	bool serviceCallSuccess = false;
	lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response = nullptr;

	if (status == std::future_status::ready) {
		try {
			lifecycle_msgs::srv::GetState::Response::SharedPtr mutableResponse = futureState.get();
			response = mutableResponse;
			if (response) {
				serviceCallSuccess = true;
			}
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Exception getting initial state future result for '%s': %s",
					this->mpLifecycleNodeNameToManage.c_str(), e.what());
		}
	} else if (status == std::future_status::timeout) {
		RCLCPP_WARN(this->get_logger(), "Timeout waiting for initial state future for node '%s'",
				this->mpLifecycleNodeNameToManage.c_str());
	} else { // INTERRUPTED or other error
		RCLCPP_ERROR(this->get_logger(), "Initial state future for node '%s' was interrupted or error. Return code: %d",
				this->mpLifecycleNodeNameToManage.c_str(), static_cast<int>(status));
	}
	this->OnVideoLoggerGetStateResponse("initial_constructor_get_state", serviceCallSuccess, response);


	this->SetupROSInterfaces();

	RCLCPP_INFO(this->get_logger(), "VideoLoggingDriver derived constructor finished.");
}

VideoLoggingDriver::~VideoLoggingDriver()
{
	RCLCPP_INFO(this->get_logger(), "VideoLoggingDriver destructor...");
	this->AttemptDriverSpecificGracefulShutdown();
}

void VideoLoggingDriver::InitializeDriverParameters()
{
	this->declare_parameter<std::string>("lifecycle_node_to_manage", DRIVER_DEFAULT_LIFECYCLE_NODE_TO_MANAGE);
	this->declare_parameter<std::string>("ap_status_topic", DRIVER_DEFAULT_AP_STATUS_TOPIC);

	this->get_parameter("lifecycle_node_to_manage", this->mpLifecycleNodeNameToManage);
	this->get_parameter("ap_status_topic", this->mpAPStatusTopic); // Renamed
	RCLCPP_INFO(this->get_logger(), "Driver will manage node: '%s'", this->mpLifecycleNodeNameToManage.c_str());
	RCLCPP_INFO(this->get_logger(), "Driver listening to status on: '%s'", this->mpAPStatusTopic.c_str());
}

void VideoLoggingDriver::SetupROSInterfaces() // Renamed
{
	rclcpp::SubscriptionOptions subOptions;
	subOptions.callback_group = mpCallbackGroupReentrant; 

	this->mpStatusSubscriber = this->create_subscription<ardupilot_msgs::msg::Status>(
			this->mpAPStatusTopic, // Renamed variable used here
			10, 
			std::bind(&VideoLoggingDriver::StatusCallback, this, std::placeholders::_1),
			subOptions); 
	RCLCPP_INFO(this->get_logger(), "Status subscriber created on topic '%s'.", this->mpAPStatusTopic.c_str());
}


void VideoLoggingDriver::StatusCallback(const ardupilot_msgs::msg::Status::ConstSharedPtr msg)
{
	this->mpStatusMessageCounter++;
	RCLCPP_INFO(this->get_logger(), "--- Status Callback Start (Msg #%d, armed: %d, flying: %d) ---",
			this->mpStatusMessageCounter, msg->armed, msg->flying);

	bool currentArmed = msg->armed;
	bool currentFlying = msg->flying;

	uint8_t transitionToAttempt = 0; 
	bool triggerTransition = false;
	bool isChainedOperation = false; 

	if (currentArmed && !this->mpPreviousArmedStatus) {
		RCLCPP_INFO(this->get_logger(), "UAV is ARMED. Target: configure '%s'.", this->mpLifecycleNodeNameToManage.c_str());
		if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED || this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
			transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
			triggerTransition = true;
		} else { 
			RCLCPP_INFO(this->get_logger(), "Node '%s' is in state '%s', no configure action needed for arming.", 
					this->mpLifecycleNodeNameToManage.c_str(), GetStateLabel(this->mpVideoLoggerKnownState).c_str());
		}
	} else if (currentFlying && !this->mpPreviousFlyingStatus && currentArmed) {
		RCLCPP_INFO(this->get_logger(), "UAV is FLYING and ARMED. Target: activate '%s'.", this->mpLifecycleNodeNameToManage.c_str());
		if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
			transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
			triggerTransition = true;
		} else if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
			RCLCPP_WARN(this->get_logger(), "'%s' is unconfigured. Will attempt to configure then activate.", this->mpLifecycleNodeNameToManage.c_str());
			isChainedOperation = true; 
			std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> configureFuture = 
				this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
			this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, configureFuture, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
		} else { 
			RCLCPP_INFO(this->get_logger(), "Node '%s' is in state '%s', no activate action needed for flying.", 
					this->mpLifecycleNodeNameToManage.c_str(), GetStateLabel(this->mpVideoLoggerKnownState).c_str());
		}
	} else if (!currentFlying && this->mpPreviousFlyingStatus) {
		RCLCPP_INFO(this->get_logger(), "UAV has STOPPED FLYING. Target: deactivate and cleanup '%s'.", this->mpLifecycleNodeNameToManage.c_str());
		if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
			isChainedOperation = true; 
			std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> deactivateFuture = 
				this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
			this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, deactivateFuture, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
		} else if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
			RCLCPP_INFO(this->get_logger(), "Node '%s' is Inactive. Directly attempting Cleanup.", this->mpLifecycleNodeNameToManage.c_str());
			transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
			triggerTransition = true; 
		} else { 
			RCLCPP_INFO(this->get_logger(), "Node '%s' is in state '%s', no deactivate/cleanup action needed for stopping flight.", 
					this->mpLifecycleNodeNameToManage.c_str(), GetStateLabel(this->mpVideoLoggerKnownState).c_str());
		}
	}

	if (triggerTransition && !isChainedOperation && transitionToAttempt != 0) {
		std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> futureResult = 
			this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, transitionToAttempt);
		this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, futureResult, transitionToAttempt);
	}

	std::unique_lock<std::mutex> lock(mpcStatusMutex); 
	this->mpPreviousArmedStatus = currentArmed;
	this->mpPreviousFlyingStatus = currentFlying;
	RCLCPP_INFO(this->get_logger(), "--- Status Callback End (Msg #%d) ---", this->mpStatusMessageCounter);
}

void VideoLoggingDriver::OnVideoLoggerChangeStateResponse(
		uint8_t attemptedTransitionId,
		bool success, 
		lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response)
{
	std::string transitionLabel = GetTransitionLabel(attemptedTransitionId); 

	RCLCPP_INFO(this->get_logger(), "OnVideoLoggerChangeStateResponse for transition '%s', service call success: %d", 
			transitionLabel.c_str(), success);
	std::unique_lock<std::mutex> lock(mpcVideoLoggerStateMutex); 
	if (success && response && response->success) {
		RCLCPP_INFO(this->get_logger(), "Transition '%s' for node '%s' SUCCEEDED.", 
				transitionLabel.c_str(), this->mpLifecycleNodeNameToManage.c_str());

		if (attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) {
			this->mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
			// Check if UAV is already flying to chain activate
			// Note: mpPreviousFlyingStatus reflects the status *when StatusCallback was last called*.
			// For more robust chaining, the decision to chain might need to be based on a more current state
			// or passed through the task system. For now, this uses the last known status from StatusCallback.
			if (this->mpPreviousFlyingStatus && this->mpPreviousArmedStatus) { 
				RCLCPP_INFO(this->get_logger(), "Configure successful for '%s', UAV is flying, now attempting Activate.", this->mpLifecycleNodeNameToManage.c_str());
				std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> activateFuture = 
					this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
				this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, activateFuture, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
			}
		} else if (attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
			this->mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
		} else if (attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
			this->mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
			RCLCPP_INFO(this->get_logger(), "Deactivate successful for '%s', now attempting Cleanup.", this->mpLifecycleNodeNameToManage.c_str());
			std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> cleanupFuture = 
				this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
			this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, cleanupFuture, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
		} else if (attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP) {
			this->mpVideoLoggerKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
		}
		RCLCPP_INFO(this->get_logger(), "Node '%s' is now assumed to be in state: %s",
				this->mpLifecycleNodeNameToManage.c_str(), this->GetStateLabel(this->mpVideoLoggerKnownState).c_str());
	} else {
		RCLCPP_ERROR(this->get_logger(), "Transition '%s' for node '%s' FAILED (service call success: %d, response valid: %d, transition success in response: %d).", 
				transitionLabel.c_str(), this->mpLifecycleNodeNameToManage.c_str(), 
				success, (response != nullptr), (response ? response->success : false) );

		std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr> getStateFuture = this->AsyncGetNodeState(this->mpLifecycleNodeNameToManage);
		this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, getStateFuture, "after_failed_change_state");
	}
}

void VideoLoggingDriver::OnVideoLoggerGetStateResponse(
		const std::string& context,
		bool success, 
		lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response)
{
	RCLCPP_INFO(this->get_logger(), "OnVideoLoggerGetStateResponse for node '%s', context: '%s', success: %d", 
			this->mpLifecycleNodeNameToManage.c_str(), context.c_str(), success);

	std::unique_lock<std::mutex> lock(mpcVideoLoggerStateMutex); 
	if (success && response) {
		this->mpVideoLoggerKnownState = response->current_state.id;
		RCLCPP_INFO(this->get_logger(), "Actual state of '%s' (%s): %s",
				this->mpLifecycleNodeNameToManage.c_str(), context.c_str(), this->GetStateLabel(this->mpVideoLoggerKnownState).c_str());
	} else {
		RCLCPP_ERROR(this->get_logger(), "Failed to get state of '%s' (%s) (service call success: %d, response valid: %d).", 
				this->mpLifecycleNodeNameToManage.c_str(), context.c_str(), success, (response != nullptr));
	}
}


void VideoLoggingDriver::AttemptDriverSpecificGracefulShutdown()
{
	RCLCPP_INFO(this->get_logger(), "Attempting graceful shutdown of managed node '%s'...", this->mpLifecycleNodeNameToManage.c_str());

	if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		RCLCPP_INFO(this->get_logger(), "Driver shutting down: Attempting to deactivate '%s' synchronously.", this->mpLifecycleNodeNameToManage.c_str());
		std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> futureState = 
			this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
		if (futureState.valid() && futureState.wait_for(this->mpServiceCallTimeoutMs) == std::future_status::ready) {
			try {
				lifecycle_msgs::srv::ChangeState::Response::SharedPtr result = futureState.get();
				if (result && result->success) {
					RCLCPP_INFO(this->get_logger(), 
							"Node '%s' deactivated during shutdown. Now attempting cleanup synchronously.",
							this->mpLifecycleNodeNameToManage.c_str());
					std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> cleanupFuture = 
						this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
					if(cleanupFuture.valid() && cleanupFuture.wait_for(this->mpServiceCallTimeoutMs) == std::future_status::ready){
						lifecycle_msgs::srv::ChangeState::Response::SharedPtr cleanupResult = cleanupFuture.get();
						if(cleanupResult && cleanupResult->success){
							RCLCPP_INFO(this->get_logger(), "Node '%s' cleaned up during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
						} else {
							RCLCPP_WARN(this->get_logger(), "Cleanup failed for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
						}
					} else {
						RCLCPP_WARN(this->get_logger(), "Cleanup timed out for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
					}
				} else {
					RCLCPP_WARN(this->get_logger(), "Deactivation failed for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
				}
			} catch(const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Exception during shutdown deactivation: %s", e.what());
			}
		} else {
			RCLCPP_WARN(this->get_logger(), "Deactivation timed out or future invalid for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
		}
	} else if (this->mpVideoLoggerKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
		RCLCPP_INFO(this->get_logger(), "Driver shutting down: Attempting to cleanup '%s' synchronously.", this->mpLifecycleNodeNameToManage.c_str());
		std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> cleanupFuture = 
			this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
		if(cleanupFuture.valid() && cleanupFuture.wait_for(this->mpServiceCallTimeoutMs) == std::future_status::ready){
			try{
				lifecycle_msgs::srv::ChangeState::Response::SharedPtr cleanupResult = cleanupFuture.get();
				if(cleanupResult && cleanupResult->success){
					RCLCPP_INFO(this->get_logger(), "Node '%s' cleaned up during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
				} else {
					RCLCPP_WARN(this->get_logger(), "Cleanup failed for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
				}
			} catch(const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Exception during shutdown cleanup: %s", e.what());
			}
		} else {
			RCLCPP_WARN(this->get_logger(), "Cleanup timed out or future invalid for '%s' during shutdown.", this->mpLifecycleNodeNameToManage.c_str());
		}
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<VideoLoggingDriver> videoLoggingDriverNode = std::make_shared<VideoLoggingDriver>(options);

	rclcpp::executors::MultiThreadedExecutor executor; 
	executor.add_node(videoLoggingDriverNode); 

	RCLCPP_INFO(videoLoggingDriverNode->get_logger(), "Spinning VideoLoggingDriver node with MultiThreadedExecutor...");
	executor.spin(); 

	RCLCPP_INFO(videoLoggingDriverNode->get_logger(), "Spin finished. Shutting down ROS 2...");
	rclcpp::shutdown(); 
	return 0;
}


