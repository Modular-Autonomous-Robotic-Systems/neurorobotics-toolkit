#include "sensors/logger/managed_logger.h"

ManagedLogger::ManagedLogger(const rclcpp::NodeOptions & options)
	: rclcpp_lifecycle::LifecycleNode("managed_logger_node", options)
{
	this->declare_parameter<std::string>("output_bag_name", "default_bag");
	this->declare_parameter<std::vector<std::string>>("topics_to_record", {});
}

CallbackReturn ManagedLogger::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_configure, configuring the recorder...");
	try {
		InitializeParameters();
	} catch (const std::runtime_error & e) {
		RCLCPP_FATAL(this->get_logger(), "Error during configuration: %s", e.what());
		return CallbackReturn::FAILURE;
	}

	// Initialize the options as shared pointers
	mpStorageOptions = std::make_shared<rosbag2_storage::StorageOptions>();
	mpStorageOptions->uri = mpOutputBagName;
	mpStorageOptions->storage_id = "sqlite3"; // Default storage format

	mpRecordOptions = std::make_shared<rosbag2_transport::RecordOptions>();
	mpRecordOptions->all = false; // We are specifying topics
	mpRecordOptions->topics = mvpTopicsToRecord;
	mpRecordOptions->rmw_serialization_format = std::string(rmw_get_serialization_format()) ;
	// mpRecordOptions->topic_polling_interval = std::chrono::milliseconds(1);

	// // --- QoS OVERRIDE CONFIGURATION ---
	// // This is the most common reason for bags not recording data.
	// // We will create a QoS profile that is compatible with `TRANSIENT_LOCAL`
	// // publishers, which are common for sensor data and simulation status.
	// rclcpp::QoS qos_profile_sensor_data(10); // History depth of 10
	// qos_profile_sensor_data.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	// qos_profile_sensor_data.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    //
	// // Apply this override to all topics specified for recording.
	// for (const auto & topic_name : mvpTopicsToRecord) {
	// 	mpRecordOptions->topic_qos_profile_overrides.emplace(topic_name, qos_profile_sensor_data);
	// }
	// // --- END QoS OVERRIDE ---

	RCLCPP_INFO(this->get_logger(), "Configuration successful.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedLogger::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_activate, activating the recorder...");

	try {
		std::unique_ptr<rosbag2_cpp::Writer> writer = rosbag2_transport::ReaderWriterFactory::make_writer(*mpRecordOptions);
		mpWriter = std::move(writer);

		rclcpp::NodeOptions options;
		std::string recorder_name = std::string(this->get_name()) + "_internal";
		mpRecorder = std::make_shared<rosbag2_transport::Recorder>(
				mpWriter, *mpStorageOptions, *mpRecordOptions, recorder_name, options);
		mpRecorderExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
		mpRecorderExecutor->add_node(mpRecorder);
		mpRecordThread = std::make_shared<std::thread>([this]() {
			mpRecorderExecutor->spin();
		});
		mpRecorder->record();
	} catch (const std::exception & e) {
		RCLCPP_FATAL(this->get_logger(), "Error during activation: %s", e.what());
		if (mpRecorder) {
			mpRecorder->stop();
		}
		if (mpRecorderExecutor) {
			// 1. Cancel the executor, which will stop the spin() call in the thread.
			mpRecorderExecutor->cancel();
		}

		if (mpRecordThread && mpRecordThread->joinable()) {
			// 2. Join the thread to wait for it to finish gracefully.
			mpRecordThread->join();
		}

		if (mpRecorder && mpRecorderExecutor) {
			// 3. Remove the node from the executor to prepare for a potential reactivation.
			mpRecorderExecutor->remove_node(mpRecorder);
		}
		mpRecorder.reset();
		mpWriter.reset();
		mpRecorderExecutor.reset();
		mpRecordThread.reset();
		return CallbackReturn::FAILURE;
	}

	RCLCPP_INFO(this->get_logger(), "Activation successful. Recording started.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedLogger::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_deactivate, stopping the recorder...");
	if (mpRecorder) {
		mpRecorder->stop();
	}
	if (mpRecorderExecutor) {
		// 1. Cancel the executor, which will stop the spin() call in the thread.
		mpRecorderExecutor->cancel();
	}

	if (mpRecordThread && mpRecordThread->joinable()) {
		// 2. Join the thread to wait for it to finish gracefully.
		mpRecordThread->join();
	}

	if (mpRecorder && mpRecorderExecutor) {
		// 3. Remove the node from the executor to prepare for a potential reactivation.
		mpRecorderExecutor->remove_node(mpRecorder);
	}
	RCLCPP_INFO(this->get_logger(), "Deactivation successful. Recording stopped.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedLogger::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_cleanup, cleaning up resources...");
	// Reset all pointers to release their resources.
	mpRecorder.reset();
	mpWriter.reset();
	mpStorageOptions.reset();
	mpRecordOptions.reset();
	mpRecorderExecutor.reset();
	mpRecordThread.reset();
	RCLCPP_INFO(this->get_logger(), "Cleanup successful.");
	return CallbackReturn::SUCCESS;
}

void ManagedLogger::InitializeParameters()
{
	this->get_parameter("output_bag_name", mpOutputBagName);
	this->get_parameter("topics_to_record", mvpTopicsToRecord);

	if (mvpTopicsToRecord.empty()) {
		RCLCPP_ERROR(this->get_logger(), "Parameter 'topics_to_record' is not set or is empty.");
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<ManagedLogger> recorder_node =
		std::make_shared<ManagedLogger>(options);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(recorder_node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
