#ifndef MANAGED_RECORDER_HPP_
#define MANAGED_RECORDER_HPP_
#include "sensors/common/utils.h"
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

class ManagedLogger : public rclcpp_lifecycle::LifecycleNode
{
	public:
		/**
		 * @brief Construct a new Managed Recorder object.
		 * @param options The NodeOptions for this lifecycle node, passed from the component manager.
		 */
		explicit ManagedLogger(const rclcpp::NodeOptions & options);

		std::shared_ptr<rosbag2_transport::Recorder> GetRecorderNode(){return mpRecorder;}

	private:
		/**
		 * @brief Lifecycle callback for the 'configuring' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful configuration.
		 * @return CallbackReturn::FAILURE on configuration failure.
		 */
		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'activating' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful activation.
		 * @return CallbackReturn::FAILURE on activation failure.
		 */
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'deactivating' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful deactivation.
		 * @return CallbackReturn::FAILURE on deactivation failure.
		 */
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'cleaning up' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful cleanup.
		 * @return CallbackReturn::FAILURE on cleanup failure.
		 */
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Reads and validates parameters from the parameter server.
		 */
		void InitializeParameters();

		/// @brief A shared pointer to the underlying rosbag2 C++ writer instance.
		std::shared_ptr<rosbag2_cpp::Writer> mpWriter;

		/// @brief A shared pointer to the main rosbag2 recorder transport object.
		std::shared_ptr<rosbag2_transport::Recorder> mpRecorder;

		/// @brief A shared pointer to the storage options for the bag file.
		std::shared_ptr<rosbag2_storage::StorageOptions> mpStorageOptions;

		/// @brief A shared pointer to the recording options.
		std::shared_ptr<rosbag2_transport::RecordOptions> mpRecordOptions;

		/// @brief The URI (path) for the output bag file.
		std::string mpOutputBagName;

		/// @brief A vector of topic names to be recorded.
		std::vector<std::string> mvpTopicsToRecord;

		/// @brief A dedicated executor to spin the internal recorder node.
		std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> mpRecorderExecutor;

		/// @brief The thread that runs the recorder's executor.
		std::shared_ptr<std::thread> mpRecordThread;
};

#endif // MANAGED_RECORDER_HPP_
