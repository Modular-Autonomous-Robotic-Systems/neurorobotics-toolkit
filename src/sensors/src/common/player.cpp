#include "sensors/common/player.h"

ManagedReader::ManagedReader(const rclcpp::NodeOptions & options)
	: rclcpp_lifecycle::LifecycleNode("managed_reader", options)
{
	this->declare_parameter<std::string>("input_bag_name", "");
	this->declare_parameter<std::vector<std::string>>("topics_to_publish", {});
}

CallbackReturn ManagedReader::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_configure, configuring the reader...");
	try {
		InitializeParameters();
	} catch (const std::runtime_error & e) {
		RCLCPP_FATAL(this->get_logger(), "Error during configuration: %s", e.what());
		return CallbackReturn::FAILURE;
	}

	mpStorageOptions = std::make_shared<rosbag2_storage::StorageOptions>();
	mpStorageOptions->uri = mpInputBagName;
	mpStorageOptions->storage_id = "sqlite3";

	mpPlayOptions = std::make_shared<rosbag2_transport::PlayOptions>();
	if (!mvpTopicsToPublish.empty()) {
		mpPlayOptions->topics_to_filter = mvpTopicsToPublish;
	}

	RCLCPP_INFO(this->get_logger(), "Configuration successful.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedReader::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_activate, activating the reader...");

	try {
		mpPlayer = std::make_shared<rosbag2_transport::Player>(
				*mpStorageOptions, *mpPlayOptions, "managed_reader_node");

		mpPlayer->play();
	} catch (const std::exception & e) {
		RCLCPP_FATAL(this->get_logger(), "Error during activation: %s", e.what());
		return CallbackReturn::FAILURE;
	}

	RCLCPP_INFO(this->get_logger(), "Activation successful. Playback started.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedReader::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_deactivate, stopping the reader...");
	if (mpPlayer) {
		mpPlayer->pause();
	}
	RCLCPP_INFO(this->get_logger(), "Deactivation successful. Playback stopped.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedReader::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "In on_cleanup, cleaning up resources...");
	// Reset all pointers to release their resources.
	mpPlayer.reset();
	mpStorageOptions.reset();
	mpPlayOptions.reset();
	RCLCPP_INFO(this->get_logger(), "Cleanup successful.");
	return CallbackReturn::SUCCESS;
}

void ManagedReader::InitializeParameters()
{
	this->get_parameter("input_bag_name", mpInputBagName);
	this->get_parameter("topics_to_publish", mvpTopicsToPublish);

	if (mpInputBagName.empty()) {
		throw std::runtime_error("Parameter 'input_bag_name' must be set.");
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<ManagedReader> reader_node =
		std::make_shared<ManagedReader>(options);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(reader_node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
