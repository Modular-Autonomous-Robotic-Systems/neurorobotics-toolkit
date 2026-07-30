#include "controllers/logging/video_controller.h"

#include <map>
#include <utility>

using namespace std::chrono_literals;

std::string SupportedAPTypesList() {
    std::string list;
    for (const std::string& apType : DRIVER_SUPPORTED_AP_TYPES) {
        if (!list.empty()) {
            list += ", ";
        }
        list += "'" + apType + "'";
    }
    return list;
}

const std::map<std::pair<uint8_t, uint8_t>, uint8_t> TRANSITION_STEP = {
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED},
     lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE},
     lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE},
    {{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED},
     lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE},
};

VideoLoggingDriver::VideoLoggingDriver(const rclcpp::NodeOptions& options)
    : LifecycleControllerBase("video_logging_driver", options) {
    RCLCPP_INFO(this->get_logger(),
                "VideoLoggingDriver derived constructor starting...");

    this->InitializeDriverParameters();

    LifecycleControllerBase::ChangeStateCallbackType changeCb = std::bind(
        &VideoLoggingDriver::OnVideoLoggerChangeStateResponse, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    LifecycleControllerBase::GetStateCallbackType getCb = std::bind(
        &VideoLoggingDriver::OnVideoLoggerGetStateResponse, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    if (!this->RegisterNode(this->mpLifecycleNodeNameToManage, changeCb,
                            getCb)) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Failed to register node '%s' with base controller. Exiting.",
            this->mpLifecycleNodeNameToManage.c_str());
        rclcpp::shutdown();
        return;
    }

    this->WaitForAllRegisteredServices();

    this->mpInitialStateTimer = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&VideoLoggingDriver::QueryInitialState, this),
        this->mpCallbackGroupReentrant);

    this->SetupROSInterfaces();

    RCLCPP_INFO(this->get_logger(),
                "VideoLoggingDriver derived constructor finished.");
}

VideoLoggingDriver::~VideoLoggingDriver() {
    RCLCPP_INFO(this->get_logger(), "VideoLoggingDriver destructor...");
    this->AttemptDriverSpecificGracefulShutdown();
}

void VideoLoggingDriver::QueryInitialState() {
    this->mpInitialStateTimer->cancel();

    RCLCPP_INFO(this->get_logger(), "Getting initial state for '%s'...",
                this->mpLifecycleNodeNameToManage.c_str());

    std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr>
        futureState =
            this->AsyncGetNodeState(this->mpLifecycleNodeNameToManage);

    this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage,
                                            futureState,
                                            "initial_constructor_get_state");
}

void VideoLoggingDriver::InitializeDriverParameters() {
    this->declare_parameter<std::string>(
        "lifecycle_node_to_manage", DRIVER_DEFAULT_LIFECYCLE_NODE_TO_MANAGE);
    this->declare_parameter<std::string>("ap_status_topic",
                                         DRIVER_DEFAULT_AP_STATUS_TOPIC);
    this->declare_parameter<std::string>("ap_type", DRIVER_DEFAULT_AP_TYPE);
    this->declare_parameter<std::string>("start_topic",
                                         START_SUBSCRIBER_TOPIC_DEFAULT);
    this->declare_parameter<std::string>("stop_topic",
                                         STOP_SUBSCRIBER_TOPIC_DEFAULT);

    this->get_parameter("lifecycle_node_to_manage",
                        this->mpLifecycleNodeNameToManage);
    this->get_parameter("ap_status_topic", this->mpAPStatusTopic);  // Renamed
    this->get_parameter("ap_type", this->mpAPType);
    this->get_parameter("start_topic", this->mpStartSubscriberTopic);
    this->get_parameter("stop_topic", this->mpStopSubscriberTopic);
    RCLCPP_INFO(this->get_logger(), "Driver will manage node: '%s'",
                this->mpLifecycleNodeNameToManage.c_str());
    RCLCPP_INFO(this->get_logger(), "Driver listening to status on: '%s'",
                this->mpAPStatusTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Driver listening to AP Type: '%s'",
                this->mpAPType.c_str());
}

void VideoLoggingDriver::SetupROSInterfaces()  // Renamed
{
    rclcpp::SubscriptionOptions subOptions;
    subOptions.callback_group = mpCallbackGroupReentrant;

    this->mpStartSubscriber = this->create_subscription<std_msgs::msg::String>(
        this->mpStartSubscriberTopic, 10,
        std::bind(&VideoLoggingDriver::StartCallback, this,
                  std::placeholders::_1),
        subOptions);

    this->mpStopSubscriber = this->create_subscription<std_msgs::msg::String>(
        this->mpStopSubscriberTopic, 10,
        std::bind(&VideoLoggingDriver::StopCallback, this,
                  std::placeholders::_1),
        subOptions);

    if (this->mpAPType == "ardupilot") {
        this->mpArduPilotStatusSubscriber =
            this->create_subscription<ardupilot_msgs::msg::Status>(
                this->mpAPStatusTopic,  // Renamed variable used here
                10,
                std::bind(&VideoLoggingDriver::ArduPilotStatusCallback, this,
                          std::placeholders::_1),
                subOptions);
        RCLCPP_INFO(this->get_logger(),
                    "ArduPilot status subscriber created on topic '%s'.",
                    this->mpAPStatusTopic.c_str());
    } else if (this->mpAPType == "tello") {
        this->mpTelloStatusSubscriber =
            this->create_subscription<std_msgs::msg::String>(
                this->mpAPStatusTopic,  // Renamed variable used here
                10,
                std::bind(&VideoLoggingDriver::TelloStatusCallback, this,
                          std::placeholders::_1),
                subOptions);
        RCLCPP_INFO(this->get_logger(),
                    "Tello status subscriber created on topic '%s'.",
                    this->mpAPStatusTopic.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Unrecognised ap_type '%s'; no status subscriber created, "
                    "so arm-triggered logging is disabled. Supported values: "
                    "%s.",
                    this->mpAPType.c_str(), SupportedAPTypesList().c_str());
    }
}

void VideoLoggingDriver::UpdateTargetState(bool currentArmed,
                                           bool currentFlying) {
    if (currentArmed && !this->mpPreviousArmedStatus) {
        RCLCPP_INFO(this->get_logger(),
                    "UAV is ARMED. Target for '%s' is ACTIVE.",
                    this->mpLifecycleNodeNameToManage.c_str());
        this->mpTargetState = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    } else if (currentFlying && currentArmed) {
        this->mpTargetState = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    } else if (!currentFlying && this->mpPreviousFlyingStatus) {
        RCLCPP_INFO(this->get_logger(),
                    "UAV has STOPPED FLYING. Target for '%s' is UNCONFIGURED.",
                    this->mpLifecycleNodeNameToManage.c_str());
        this->mpTargetState =
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    }
}

void VideoLoggingDriver::Reconcile() {
    uint8_t transitionToAttempt = 0;
    uint8_t knownState = 0;
    uint8_t targetState = 0;

    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);
        if (this->mpTransitionInFlight) {
            return;
        }
        knownState = this->mpVideoLoggerKnownState;
        targetState = this->mpTargetState;
        if (targetState == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
            return;
        }
        if (knownState == targetState) {
            return;
        }
        std::map<std::pair<uint8_t, uint8_t>, uint8_t>::const_iterator step =
            TRANSITION_STEP.find(std::make_pair(knownState, targetState));
        if (step == TRANSITION_STEP.end()) {
            RCLCPP_INFO(this->get_logger(),
                        "Node '%s' is in state '%s' and no single lifecycle "
                        "step reaches '%s', so no action is taken.",
                        this->mpLifecycleNodeNameToManage.c_str(),
                        GetStateLabel(knownState).c_str(),
                        GetStateLabel(targetState).c_str());
            return;
        }
        transitionToAttempt = step->second;
        this->mpTransitionInFlight = true;
    }

    RCLCPP_INFO(this->get_logger(), "Requesting '%s' on '%s', %s -> %s.",
                GetTransitionLabel(transitionToAttempt).c_str(),
                this->mpLifecycleNodeNameToManage.c_str(),
                GetStateLabel(knownState).c_str(),
                GetStateLabel(targetState).c_str());
    std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
        futureResult = this->AsyncCallChangeState(
            this->mpLifecycleNodeNameToManage, transitionToAttempt);
    this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage,
                                            futureResult, transitionToAttempt);
}

void VideoLoggingDriver::StartCallback(
    const std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Manual start requested, target for '%s' is ACTIVE.",
                this->mpLifecycleNodeNameToManage.c_str());
    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);
        this->mpTargetState = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }
    this->Reconcile();
}

void VideoLoggingDriver::StopCallback(
    const std_msgs::msg::String::ConstSharedPtr msg) {}

void VideoLoggingDriver::ArduPilotStatusCallback(
    const ardupilot_msgs::msg::Status::ConstSharedPtr msg) {
    this->mpStatusMessageCounter++;
    RCLCPP_DEBUG(
        this->get_logger(),
        "--- Status Callback Start (Msg #%d, armed: %d, flying: %d) ---",
        this->mpStatusMessageCounter, msg->armed, msg->flying);

    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);
        this->UpdateTargetState(msg->armed, msg->flying);
        this->mpPreviousArmedStatus = msg->armed;
        this->mpPreviousFlyingStatus = msg->flying;
    }

    this->Reconcile();

    RCLCPP_DEBUG(this->get_logger(), "--- Status Callback End (Msg #%d) ---",
                 this->mpStatusMessageCounter);
}

void VideoLoggingDriver::TelloStatusCallback(
    const std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received Tello Status %s",
                 msg->data.c_str());
    this->mpStatusMessageCounter++;
    bool currentArmed = false;
    bool currentFlying = false;

    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);

        if (msg->data == "taking_off") {
            currentArmed = true;
            currentFlying = true;
        } else if (msg->data == "flying") {
            if (!this->mpPreviousFlyingStatus) {
                currentArmed = true;
            }
            currentFlying = true;
        } else if (msg->data == "landing") {
            currentArmed = this->mpPreviousArmedStatus;
            currentFlying = true;
        } else if (msg->data == "landed" || msg->data == "idle") {
            currentArmed = false;
            currentFlying = false;
        } else if (msg->data == "low_battery") {
            RCLCPP_WARN(this->get_logger(),
                        "Tello reports low battery; leaving recording state "
                        "unchanged.");
            return;
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Unrecognised tello_state '%s'; ignoring.",
                        msg->data.c_str());
            return;
        }

        this->UpdateTargetState(currentArmed, currentFlying);
        this->mpPreviousArmedStatus = currentArmed;
        this->mpPreviousFlyingStatus = currentFlying;
    }

    this->Reconcile();

    RCLCPP_DEBUG(this->get_logger(), "--- Status Callback End (Msg #%d) ---",
                 this->mpStatusMessageCounter);
}

void VideoLoggingDriver::OnVideoLoggerChangeStateResponse(
    uint8_t attemptedTransitionId, bool success,
    lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response) {
    std::string transitionLabel = GetTransitionLabel(attemptedTransitionId);

    RCLCPP_INFO(this->get_logger(),
                "OnVideoLoggerChangeStateResponse for transition '%s', service "
                "call success: %d",
                transitionLabel.c_str(), success);

    bool transitionSucceeded = success && response && response->success;
    uint8_t resultingState = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);
        this->mpTransitionInFlight = false;
        if (transitionSucceeded) {
            if (attemptedTransitionId ==
                lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) {
                this->mpVideoLoggerKnownState =
                    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
            } else if (attemptedTransitionId ==
                       lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
                this->mpVideoLoggerKnownState =
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            } else if (attemptedTransitionId ==
                       lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
                this->mpVideoLoggerKnownState =
                    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
            } else if (attemptedTransitionId ==
                       lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP) {
                this->mpVideoLoggerKnownState =
                    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
            }
            resultingState = this->mpVideoLoggerKnownState;
        }
    }

    if (transitionSucceeded) {
        RCLCPP_INFO(
            this->get_logger(),
            "Transition '%s' for node '%s' SUCCEEDED, the node is now '%s'.",
            transitionLabel.c_str(), this->mpLifecycleNodeNameToManage.c_str(),
            GetStateLabel(resultingState).c_str());
        this->Reconcile();
        return;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Transition '%s' for node '%s' FAILED (service call success: "
                 "%d, response valid: %d, transition success in response: %d).",
                 transitionLabel.c_str(),
                 this->mpLifecycleNodeNameToManage.c_str(), success,
                 (response != nullptr), (response ? response->success : false));

    std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr>
        getStateFuture =
            this->AsyncGetNodeState(this->mpLifecycleNodeNameToManage);
    this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage,
                                            getStateFuture,
                                            "after_failed_change_state");
}

void VideoLoggingDriver::OnVideoLoggerGetStateResponse(
    const std::string& context, bool success,
    lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response) {
    RCLCPP_INFO(this->get_logger(),
                "OnVideoLoggerGetStateResponse for node '%s', context: '%s', "
                "success: %d",
                this->mpLifecycleNodeNameToManage.c_str(), context.c_str(),
                success);

    if (success && response) {
        uint8_t knownState = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        {
            std::unique_lock<std::mutex> lock(mpcStateMutex);
            this->mpVideoLoggerKnownState = response->current_state.id;
            knownState = this->mpVideoLoggerKnownState;
        }
        RCLCPP_INFO(this->get_logger(), "Actual state of '%s' (%s): %s",
                    this->mpLifecycleNodeNameToManage.c_str(), context.c_str(),
                    this->GetStateLabel(knownState).c_str());
        if (context != "after_failed_change_state") {
            this->Reconcile();
        }
    } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get state of '%s' (%s) (service call success: %d, "
            "response valid: %d).",
            this->mpLifecycleNodeNameToManage.c_str(), context.c_str(), success,
            (response != nullptr));
    }
}

void VideoLoggingDriver::AttemptDriverSpecificGracefulShutdown() {
    RCLCPP_INFO(this->get_logger(),
                "Attempting graceful shutdown of managed node '%s'...",
                this->mpLifecycleNodeNameToManage.c_str());
    uint8_t videoLoggerState;
    {
        std::unique_lock<std::mutex> lock(mpcStateMutex);
        videoLoggerState = this->mpVideoLoggerKnownState;
    }
    if (videoLoggerState == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(this->get_logger(),
                    "Driver shutting down: Attempting to deactivate '%s' "
                    "synchronously.",
                    this->mpLifecycleNodeNameToManage.c_str());
        std::shared_future<
            lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
            futureState = this->AsyncCallChangeState(
                this->mpLifecycleNodeNameToManage,
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        if (futureState.valid() &&
            futureState.wait_for(this->mpServiceCallTimeoutMs) ==
                std::future_status::ready) {
            try {
                lifecycle_msgs::srv::ChangeState::Response::SharedPtr result =
                    futureState.get();
                if (result && result->success) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Node '%s' deactivated during shutdown. Now attempting "
                        "cleanup synchronously.",
                        this->mpLifecycleNodeNameToManage.c_str());
                    std::shared_future<
                        lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
                        cleanupFuture = this->AsyncCallChangeState(
                            this->mpLifecycleNodeNameToManage,
                            lifecycle_msgs::msg::Transition::
                                TRANSITION_CLEANUP);
                    if (cleanupFuture.valid() &&
                        cleanupFuture.wait_for(this->mpServiceCallTimeoutMs) ==
                            std::future_status::ready) {
                        lifecycle_msgs::srv::ChangeState::Response::SharedPtr
                            cleanupResult = cleanupFuture.get();
                        if (cleanupResult && cleanupResult->success) {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "Node '%s' cleaned up during shutdown.",
                                this->mpLifecycleNodeNameToManage.c_str());
                        } else {
                            RCLCPP_WARN(
                                this->get_logger(),
                                "Cleanup failed for '%s' during shutdown.",
                                this->mpLifecycleNodeNameToManage.c_str());
                        }
                    } else {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Cleanup timed out for '%s' during shutdown.",
                            this->mpLifecycleNodeNameToManage.c_str());
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Deactivation failed for '%s' during shutdown.",
                                this->mpLifecycleNodeNameToManage.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                             "Exception during shutdown deactivation: %s",
                             e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Deactivation timed out or future invalid for '%s' "
                        "during shutdown.",
                        this->mpLifecycleNodeNameToManage.c_str());
        }
    } else if (videoLoggerState ==
               lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_INFO(
            this->get_logger(),
            "Driver shutting down: Attempting to cleanup '%s' synchronously.",
            this->mpLifecycleNodeNameToManage.c_str());
        std::shared_future<
            lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
            cleanupFuture = this->AsyncCallChangeState(
                this->mpLifecycleNodeNameToManage,
                lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        if (cleanupFuture.valid() &&
            cleanupFuture.wait_for(this->mpServiceCallTimeoutMs) ==
                std::future_status::ready) {
            try {
                lifecycle_msgs::srv::ChangeState::Response::SharedPtr
                    cleanupResult = cleanupFuture.get();
                if (cleanupResult && cleanupResult->success) {
                    RCLCPP_INFO(this->get_logger(),
                                "Node '%s' cleaned up during shutdown.",
                                this->mpLifecycleNodeNameToManage.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Cleanup failed for '%s' during shutdown.",
                                this->mpLifecycleNodeNameToManage.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                             "Exception during shutdown cleanup: %s", e.what());
            }
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Cleanup timed out or future invalid for '%s' during shutdown.",
                this->mpLifecycleNodeNameToManage.c_str());
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::shared_ptr<VideoLoggingDriver> videoLoggingDriverNode =
        std::make_shared<VideoLoggingDriver>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(videoLoggingDriverNode);

    RCLCPP_INFO(
        videoLoggingDriverNode->get_logger(),
        "Spinning VideoLoggingDriver node with MultiThreadedExecutor...");
    executor.spin();

    RCLCPP_INFO(videoLoggingDriverNode->get_logger(),
                "Spin finished. Shutting down ROS 2...");
    rclcpp::shutdown();
    return 0;
}
