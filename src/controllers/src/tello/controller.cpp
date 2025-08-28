#include <controllers/tello/controller.h>

using namespace std::chrono_literals;

TelloControllerNode::TelloControllerNode(const rclcpp::NodeOptions& options):LifecycleControllerBase("tello_controller_driver", options){
    //initialize time keeper
	rcl_interfaces::msg::ParameterDescriptor droneNameDesc;
	droneNameDesc.description = "Path to output file for received video";
	droneNameDesc.type = 4;
	this->declare_parameter<std::string>("drone_name", "drone1", droneNameDesc);
	mpDroneName = this->get_parameter("drone_name").as_string();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting node...");
    

    mpClock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	LifecycleControllerBase::ChangeStateCallbackType changeCb =
		std::bind(&TelloControllerNode::OnTelloDriverChangeStateResponse, this,
				std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	LifecycleControllerBase::GetStateCallbackType getCb =
		std::bind(&TelloControllerNode::OnTelloDriverGetStateResponse, this,
				std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	this->mpLifecycleNodeNameToManage = mpDroneName + "/tello_joy";
	if (!this->RegisterNode(this->mpLifecycleNodeNameToManage, changeCb, getCb)) {
		RCLCPP_FATAL(this->get_logger(), "Failed to register node '%s' with base controller. Exiting.", this->mpLifecycleNodeNameToManage.c_str());
		rclcpp::shutdown();
		return;
	}

	this->WaitForAllRegisteredServices();

    //subsription
    mpFlightDataSubsriber = this->create_subscription<tello_msgs::msg::FlightData>(
        "flight_data",
        10,
        std::bind(&TelloControllerNode::FlightDataCallback,this,std::placeholders::_1)
    );

    mpActionResponseSubscriber = this->create_subscription<std_msgs::msg::String>(
        "tello_response",
        10,
        std::bind(&TelloControllerNode::Response,this, std::placeholders::_1)
    );

    mpTelloStateSubscriber = this->create_subscription<std_msgs::msg::String>(
        "tello_state",
        10,
        std::bind(&TelloControllerNode::TelloStateCallback, this, std::placeholders::_1)
    );

    mpActionClient = this->create_client<tello_msgs::srv::TelloAction>(
        "tello_action"
    );
    
	uint8_t transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
	std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> futureResult =  this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, transitionToAttempt);
	this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, futureResult, transitionToAttempt);

    using namespace std::chrono_literals;

    // mpFlightChecker = this->create_wall_timer(15s,std::bind(&TelloControllerNode::timer_callback,this));

}

// void TelloControllerNode::timer_callback(){
//     //pre take off , check status of telemetry and frequency
//     
//     if(!mpReadyTakeOff && this->mpTelloDriverKnownState == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preflight Check");
// 		// changeTeleopState("activate");
// 		uint8_t transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
// 		std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> futureResult =  this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, transitionToAttempt);
// 		this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, futureResult, transitionToAttempt);
//
// 		mpReadyTakeOff = true;
//
// 		RCLCPP_INFO(this->get_logger(), "Camera and Image: OK");
// 		RCLCPP_INFO(this->get_logger(), "Ready To Take off...");
//
//     }
// }

void TelloControllerNode::FlightDataCallback(const tello_msgs::msg::FlightData::ConstSharedPtr &flightData){

    //will be utilized more when moving into autonomous solution
    //check if flight data empty


    if(!flightData){
        RCLCPP_ERROR(this->get_logger(), ("Got empty Flight data"));
        return;
    }
    // TODO
    // more failsafe condition, where we force emergency landing if condition are met

    if(flightData->bat<10){
		RCLCPP_WARN(this->get_logger(), "Battery less than 10%. Unable to fly");
        ActionRequestSender("land");
    }

}

//this will be used for fail safe request sender

void TelloControllerNode::ActionRequestSender(const std::string &cmd){

    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();

    request->cmd = cmd;

    mpActionRequest = cmd;

    using ServiceResponseFuture = rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture;

    // Create response callback
    auto response_callback = [this, cmd](ServiceResponseFuture future) {
        try {
            auto result = future.get();
            if (result->OK) {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully sending command %s", cmd.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to send command %s", cmd.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Exception while processing command: %s", e.what());
        }
    };

    auto future = mpActionClient->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Sent request to send command %s", cmd.c_str());

}

//i think it will not be used as the driver already handle action and responses
void TelloControllerNode::Response(const std_msgs::msg::String::ConstSharedPtr &response){

    if(!response){
        RCLCPP_ERROR(this->get_logger(), "Got empty response message");
        return;
    }
}

void TelloControllerNode::changeTeleopState(const std::string state){

    //
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    //manage teleop node state
    if(state == "configure")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    if(state == "activate")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    if(state == "deactivate")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    if(state == "clean_up")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

    using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;

    // Create response callback
    auto response_callback = [this, state](ServiceResponseFuture future) {
        try {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully changed teleop node state to %s", state.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to change teleop node state to %s", state.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Exception while processing teleop state change response: %s", e.what());
        }

    };
    
    // mpTeleopLifecycleClient->async_send_request(request, response_callback);
    RCLCPP_INFO(this->get_logger(), "Sent request to change teleop state to %s", state.c_str());

    //sleep to wait for callback return
    auto start_time = mpClock->now();
    rclcpp::Duration duration(5, 0); // 5 seconds

    rclcpp::Rate rate(10); // 10 Hz loop

    while ((mpClock->now() - start_time) < duration && rclcpp::ok()) {
        rate.sleep();  // Sleeps while allowing ROS to process callbacks
    }
}

void TelloControllerNode::TelloStateCallback(const std_msgs::msg::String::ConstSharedPtr &stateMessage){

    if(!stateMessage){
        RCLCPP_ERROR(this->get_logger(), "invalid state message");
    }
    
    mpTelloFlightState = stateMessage->data;

}
void TelloControllerNode::OnTelloDriverChangeStateResponse(
		uint8_t attemptedTransitionId,
		bool success,
		lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response){
	RCLCPP_INFO(this->get_logger(), "Received Response after tello driver state change request with transition id: %d", attemptedTransitionId);
	if(success){
		if(attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE){
			RCLCPP_INFO(this->get_logger(), "Successfully configured tello driver node");
			this->mpTelloDriverKnownState = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
			if (!mpReadyTakeOff) {
				RCLCPP_INFO(this->get_logger(), "Node not activated, attempting activation");
				uint8_t transitionToAttempt = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
				std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> futureResult =  this->AsyncCallChangeState(this->mpLifecycleNodeNameToManage, transitionToAttempt);
				this->EnqueueServiceResponseHandlerTask(this->mpLifecycleNodeNameToManage, futureResult, transitionToAttempt);

				mpReadyTakeOff = true;

				RCLCPP_INFO(this->get_logger(), "Camera and Image: OK");
				RCLCPP_INFO(this->get_logger(), "Ready To Take off...");

			}else{
				RCLCPP_ERROR(this->get_logger(), "Did not recieved any image...");
			}

		}
		else if(attemptedTransitionId == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE){
			RCLCPP_INFO(this->get_logger(), "Tello Joystick Node activated");	
		}
	}
}

void TelloControllerNode::OnTelloDriverGetStateResponse(
		const std::string& context,
		bool success,
		lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response){

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
    std::shared_ptr<TelloControllerNode> node = std::make_shared<TelloControllerNode>(options);
	rclcpp::executors::MultiThreadedExecutor executor; 
	executor.add_node(node); 
	executor.spin(); 
    rclcpp::shutdown();
    return 0;
}
