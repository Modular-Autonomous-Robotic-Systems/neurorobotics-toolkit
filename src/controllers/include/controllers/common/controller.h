#ifndef LIFECYCLE_CONTROLLER_BASE_HPP_
#define LIFECYCLE_CONTROLLER_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <future> 
#include <chrono>

// Default timeout values
const int DEFAULT_WAIT_FOR_SERVICES_TIMEOUT_MS = 1000; 
const int DEFAULT_SERVICE_CALL_TIMEOUT_MS = 10000;   

class LifecycleControllerBase : public rclcpp::Node, public std::enable_shared_from_this<LifecycleControllerBase>
{
	public:
		// Callback types for derived classes to provide (nodeName is implicit by registration)
		using ChangeStateCallbackType = std::function<void(
				uint8_t attemptedTransitionId,
				bool success,
				lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response)>;
		using GetStateCallbackType = std::function<void(
				const std::string& context,
				bool success,
				lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response)>;

		explicit LifecycleControllerBase(
				const std::string& nodeName,
				const rclcpp::NodeOptions & options,
				size_t threadPoolSize = std::thread::hardware_concurrency());
		virtual ~LifecycleControllerBase();

	protected:
		bool RegisterNode(
			const std::string& lifecycleNodeName,
			ChangeStateCallbackType changeStateCallback,
			GetStateCallbackType getStateCallback
		);
		
		std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
		AsyncCallChangeState(const std::string& lifecycleNodeName, uint8_t transitionId);

		std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr>
		AsyncGetNodeState(const std::string& lifecycleNodeName);

		void EnqueueServiceResponseHandlerTask(
			const std::string& nodeName, // Used to lookup the registered callback
			std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> future,
			uint8_t attemptedTransitionId
		);

		void EnqueueServiceResponseHandlerTask(
			const std::string& nodeName, // Used to lookup the registered callback
			std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr> future,
			const std::string& context
		);
		
		void EnqueueTask(std::function<void()> task);

		std::string GetStateLabel(uint8_t stateId);
		std::string GetTransitionLabel(uint8_t transitionId);
		
		rclcpp::CallbackGroup::SharedPtr mpCallbackGroupReentrant;
		
		std::chrono::milliseconds mpWaitForServicesTimeoutMs;
		std::chrono::milliseconds mpServiceCallTimeoutMs;

		void WaitForAllRegisteredServices(); 
	
	private:
		void ThreadPoolWorker();
		void InitializeBaseParameters();

		std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> mpChangeStateClients;
		std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> mpGetStateClients;
		std::map<std::string, ChangeStateCallbackType> mpChangeStateCallbacks; // Stores node-specific callbacks
		std::map<std::string, GetStateCallbackType> mpGetStateCallbacks;     // Stores node-specific callbacks
		std::vector<std::string> mpRegisteredNodeNames;

		std::vector<std::thread> mvpWorkerThreads;
		std::queue<std::function<void()>> mpcTaskQueue;
		std::mutex mpcQueueMutex;
		std::condition_variable mpcConditionVariable;
		bool mpcStopThreadPool = false;
};

#endif // LIFECYCLE_CONTROLLER_BASE_HPP_
