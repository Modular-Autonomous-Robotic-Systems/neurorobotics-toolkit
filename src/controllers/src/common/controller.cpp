#include "controllers/common/controller.h"


using namespace std::chrono_literals;

LifecycleControllerBase::LifecycleControllerBase(const std::string& nodeName, const rclcpp::NodeOptions & options, size_t threadPoolSize)
: Node(nodeName, options), mpcStopThreadPool(false)
{
    RCLCPP_INFO(this->get_logger(), "LifecycleControllerBase constructor for node '%s'.", nodeName.c_str());
    
    this->InitializeBaseParameters(); 

    mpCallbackGroupReentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    if (threadPoolSize == 0) {
        threadPoolSize = 1; 
    }
    RCLCPP_INFO(this->get_logger(), "Initializing thread pool with %zu threads.", threadPoolSize);
    for (size_t i = 0; i < threadPoolSize; ++i) {
        mvpWorkerThreads.emplace_back(&LifecycleControllerBase::ThreadPoolWorker, this);
    }
}

LifecycleControllerBase::~LifecycleControllerBase()
{
    RCLCPP_INFO(this->get_logger(), "LifecycleControllerBase destructor. Stopping thread pool...");
    {
        std::unique_lock<std::mutex> lock(mpcQueueMutex);
        mpcStopThreadPool = true;
    }
    mpcConditionVariable.notify_all();
    for (std::thread &worker : mvpWorkerThreads) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    RCLCPP_INFO(this->get_logger(), "Thread pool stopped.");
}

void LifecycleControllerBase::InitializeBaseParameters()
{
    this->declare_parameter<int>("wait_for_services_timeout_ms", DEFAULT_WAIT_FOR_SERVICES_TIMEOUT_MS);
    this->declare_parameter<int>("service_call_timeout_ms", DEFAULT_SERVICE_CALL_TIMEOUT_MS);

    int waitForServicesTimeoutInt;
    int serviceCallTimeoutInt;
    this->get_parameter("wait_for_services_timeout_ms", waitForServicesTimeoutInt);
    this->get_parameter("service_call_timeout_ms", serviceCallTimeoutInt);

    mpWaitForServicesTimeoutMs = std::chrono::milliseconds(waitForServicesTimeoutInt);
    mpServiceCallTimeoutMs = std::chrono::milliseconds(serviceCallTimeoutInt);

    RCLCPP_INFO(this->get_logger(), "WaitForServices Timeout: %lld ms", static_cast<long long>(mpWaitForServicesTimeoutMs.count()));
    RCLCPP_INFO(this->get_logger(), "ServiceCall Timeout: %lld ms", static_cast<long long>(mpServiceCallTimeoutMs.count()));
}

bool LifecycleControllerBase::RegisterNode(
    const std::string& lifecycleNodeName,
    ChangeStateCallbackType changeStateCallback,
    GetStateCallbackType getStateCallback)
{
    RCLCPP_INFO(this->get_logger(), "Registering lifecycle node: '%s'", lifecycleNodeName.c_str());
    if (mpChangeStateClients.count(lifecycleNodeName) || mpGetStateClients.count(lifecycleNodeName)) {
        RCLCPP_WARN(this->get_logger(), "Node '%s' is already registered.", lifecycleNodeName.c_str());
        return false;
    }

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeClient = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/" + lifecycleNodeName + "/change_state",
        rmw_qos_profile_services_default,
        mpCallbackGroupReentrant);
    mpChangeStateClients[lifecycleNodeName] = changeClient;
    mpChangeStateCallbacks[lifecycleNodeName] = changeStateCallback;

    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getClient = this->create_client<lifecycle_msgs::srv::GetState>(
        "/" + lifecycleNodeName + "/get_state",
        rmw_qos_profile_services_default,
        mpCallbackGroupReentrant);
    mpGetStateClients[lifecycleNodeName] = getClient;
    mpGetStateCallbacks[lifecycleNodeName] = getStateCallback;
    
    mpRegisteredNodeNames.push_back(lifecycleNodeName);

    RCLCPP_INFO(this->get_logger(), "Node '%s' registered with clients and specific callbacks.", lifecycleNodeName.c_str());
    return true;
}

void LifecycleControllerBase::WaitForAllRegisteredServices() {
    RCLCPP_INFO(this->get_logger(), "Waiting for all registered lifecycle services to become available (timeout per service: %lld ms)...", static_cast<long long>(mpWaitForServicesTimeoutMs.count()));
    for (const std::string& nodeName : mpRegisteredNodeNames) {
        RCLCPP_INFO(this->get_logger(), "Waiting for services of node '%s'...", nodeName.c_str());
        
        std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>::iterator changeIt = mpChangeStateClients.find(nodeName);
        if (changeIt != mpChangeStateClients.end() && changeIt->second) {
            std::string serviceName = "/" + nodeName + "/change_state";
            while(rclcpp::ok() && !changeIt->second->wait_for_service(mpWaitForServicesTimeoutMs)) {
                RCLCPP_WARN(this->get_logger(), "Service '%s' not available, waiting...", serviceName.c_str());
            }
        }
        std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>::iterator getIt = mpGetStateClients.find(nodeName);
        if (getIt != mpGetStateClients.end() && getIt->second) {
            std::string serviceName = "/" + nodeName + "/get_state";
             while(rclcpp::ok() && !getIt->second->wait_for_service(mpWaitForServicesTimeoutMs)) {
                RCLCPP_WARN(this->get_logger(), "Service '%s' not available, waiting...", serviceName.c_str());
            }
        }
    }
     if (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Finished waiting for all registered lifecycle services (or timed out for some).");
    } else {
        RCLCPP_ERROR(this->get_logger(), "rclcpp is not ok, service wait aborted for some services.");
    }
}


std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>
LifecycleControllerBase::AsyncCallChangeState(const std::string& lifecycleNodeName, uint8_t transitionId)
{
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>::iterator it = mpChangeStateClients.find(lifecycleNodeName);
    if (it == mpChangeStateClients.end() || !it->second) {
        RCLCPP_ERROR(this->get_logger(), "No ChangeState client found or client invalid for node '%s'.", lifecycleNodeName.c_str());
        std::shared_ptr<std::promise<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>> promise = 
            std::make_shared<std::promise<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>>();
        promise->set_value(nullptr); 
        return promise->get_future().share();
    }
    if (!it->second->service_is_ready()) {
         RCLCPP_WARN(this->get_logger(), "ChangeState service for node '%s' is not ready.", lifecycleNodeName.c_str());
        std::shared_ptr<std::promise<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>> promise = 
            std::make_shared<std::promise<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>>();
        promise->set_value(nullptr);
        return promise->get_future().share();
    }

    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transitionId;
    
    RCLCPP_DEBUG(this->get_logger(), "AsyncCallChangeState: Sending request for transition ID %d to node '%s'", transitionId, lifecycleNodeName.c_str());
	rclcpp::Client<lifecycle_msgs::srv::ChangeState>::FutureAndRequestId future_and_request_id = it->second->async_send_request(request);
    return future_and_request_id.future.share();
}

std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr>
LifecycleControllerBase::AsyncGetNodeState(const std::string& lifecycleNodeName)
{
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>::iterator it = mpGetStateClients.find(lifecycleNodeName);
    if (it == mpGetStateClients.end() || !it->second) {
        RCLCPP_ERROR(this->get_logger(), "No GetState client found or client invalid for node '%s'.", lifecycleNodeName.c_str());
        std::shared_ptr<std::promise<lifecycle_msgs::srv::GetState::Response::SharedPtr>> promise =
            std::make_shared<std::promise<lifecycle_msgs::srv::GetState::Response::SharedPtr>>();
        promise->set_value(nullptr);
        return promise->get_future().share();
    }
    if (!it->second->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "GetState service for node '%s' is not ready.", lifecycleNodeName.c_str());
        std::shared_ptr<std::promise<lifecycle_msgs::srv::GetState::Response::SharedPtr>> promise =
            std::make_shared<std::promise<lifecycle_msgs::srv::GetState::Response::SharedPtr>>();
        promise->set_value(nullptr);
        return promise->get_future().share();
    }
    std::shared_ptr<lifecycle_msgs::srv::GetState::Request> request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    RCLCPP_DEBUG(this->get_logger(), "AsyncGetNodeState: Sending request to node '%s'", lifecycleNodeName.c_str());
	rclcpp::Client<lifecycle_msgs::srv::GetState>::FutureAndRequestId future_and_request_id = it->second->async_send_request(request);
    return future_and_request_id.future.share();
}

void LifecycleControllerBase::EnqueueServiceResponseHandlerTask(
    const std::string& nodeName,
    std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr> future,
    uint8_t attemptedTransitionId)
{
    this->EnqueueTask([this, nodeName, future, attemptedTransitionId]() {
        RCLCPP_DEBUG(this->get_logger(), "Task: Processing ChangeState future for node '%s', transition '%s'",
            nodeName.c_str(), this->GetTransitionLabel(attemptedTransitionId).c_str());
        
        std::future_status status = future.wait_for(this->mpServiceCallTimeoutMs);
        lifecycle_msgs::srv::ChangeState::Response::ConstSharedPtr response = nullptr; 
        bool success = false;

        if (status == std::future_status::ready) {
            try {
                lifecycle_msgs::srv::ChangeState::Response::SharedPtr mutableResponse = future.get(); 
                response = mutableResponse; 
                if (response) {
                    success = response->success;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Task: Exception getting ChangeState future result for '%s': %s", nodeName.c_str(), e.what());
            }
        } else if (status == std::future_status::timeout) {
            RCLCPP_WARN(this->get_logger(), "Task: Timeout waiting for ChangeState future for node '%s', transition '%s'", 
                nodeName.c_str(), this->GetTransitionLabel(attemptedTransitionId).c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task: ChangeState future for node '%s' deferred or error.", nodeName.c_str());
        }

        std::map<std::string, ChangeStateCallbackType>::iterator cbIt = this->mpChangeStateCallbacks.find(nodeName);
        if (cbIt != this->mpChangeStateCallbacks.end() && cbIt->second) {
            cbIt->second(attemptedTransitionId, success, response);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task: No ChangeState callback registered for node '%s'", nodeName.c_str());
        }
    });
}

void LifecycleControllerBase::EnqueueServiceResponseHandlerTask(
    const std::string& nodeName,
    std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr> future,
    const std::string& context)
{
    this->EnqueueTask([this, nodeName, future, context]() {
        RCLCPP_DEBUG(this->get_logger(), "Task: Processing GetState future for node '%s', context '%s'",
            nodeName.c_str(), context.c_str());

		std::future_status status = future.wait_for(this->mpServiceCallTimeoutMs);

        lifecycle_msgs::srv::GetState::Response::ConstSharedPtr response = nullptr;
        bool success = false; 

        if (status == std::future_status::ready) {
            try {
                lifecycle_msgs::srv::GetState::Response::SharedPtr mutableResponse = future.get();
                response = mutableResponse;
                if (response) {
                    success = true; 
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Task: Exception getting GetState future result for '%s': %s", nodeName.c_str(), e.what());
            }
        } else if (status == std::future_status::timeout) {
            RCLCPP_WARN(this->get_logger(), "Task: Timeout waiting for GetState future for node '%s', context '%s'", 
                nodeName.c_str(), context.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task: GetState future for node '%s' deferred or error.", nodeName.c_str());
        }
        
        std::map<std::string, GetStateCallbackType>::iterator cbIt = this->mpGetStateCallbacks.find(nodeName);
        if (cbIt != this->mpGetStateCallbacks.end() && cbIt->second) {
            cbIt->second(context, success, response);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task: No GetState callback registered for node '%s'", nodeName.c_str());
        }
    });
}


void LifecycleControllerBase::ThreadPoolWorker()
{
    RCLCPP_DEBUG(this->get_logger(), "Thread pool worker started. Thread ID: %s", 
        std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
    while (rclcpp::ok()) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mpcQueueMutex);
            mpcConditionVariable.wait(lock, [this] { return mpcStopThreadPool || !mpcTaskQueue.empty(); });
            if (mpcStopThreadPool && mpcTaskQueue.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Thread pool worker stopping. Thread ID: %s",
                    std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
                return;
            }
            if (mpcTaskQueue.empty()) { 
                continue;
            }
            task = std::move(mpcTaskQueue.front());
            mpcTaskQueue.pop();
        }
        RCLCPP_DEBUG(this->get_logger(), "Thread pool worker picked up a task. Thread ID: %s. Tasks remaining: %zu",
            std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str(), mpcTaskQueue.size());
        // try {
            task(); 
        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(this->get_logger(), "Exception caught in thread pool worker: %s. Thread ID: %s", 
        //         e.what(), std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
        // } catch (...) {
        //     RCLCPP_ERROR(this->get_logger(), "Unknown exception caught in thread pool worker. Thread ID: %s",
        //         std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
        // }
    }
    RCLCPP_INFO(this->get_logger(), "Thread pool worker exiting due to rclcpp not ok. Thread ID: %s",
        std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
}

void LifecycleControllerBase::EnqueueTask(std::function<void()> task)
{
    if (!rclcpp::ok() || mpcStopThreadPool) { 
        RCLCPP_WARN(this->get_logger(), "rclcpp not ok or thread pool is stopping. Task not enqueued.");
        return;
    }
    {
        std::unique_lock<std::mutex> lock(mpcQueueMutex);
        mpcTaskQueue.push(std::move(task));
    }
    mpcConditionVariable.notify_one();
    RCLCPP_DEBUG(this->get_logger(), "Task enqueued. Queue size: %zu", mpcTaskQueue.size());
}

std::string LifecycleControllerBase::GetStateLabel(uint8_t stateId) {
    switch (stateId) {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN: return "Unknown";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: return "Unconfigured";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE: return "Inactive";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE: return "Active";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED: return "Finalized";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING: return "Configuring";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP: return "CleaningUp";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN: return "ShuttingDown";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING: return "Activating";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING: return "Deactivating";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING: return "ErrorProcessing";
        default: return "StateId(" + std::to_string(stateId) + ")";
    }
}

std::string LifecycleControllerBase::GetTransitionLabel(uint8_t transitionId) {
     switch (transitionId) {
        case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE: return "Configure";
        case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP: return "Cleanup";
        case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE: return "Activate";
        case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE: return "Deactivate";
        case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: return "UnconfiguredShutdown";
        case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN: return "InactiveShutdown";
        case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN: return "ActiveShutdown";
        case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY: return "Destroy";
        default: return "TransitionId(" + std::to_string(transitionId) + ")";
    }
}
