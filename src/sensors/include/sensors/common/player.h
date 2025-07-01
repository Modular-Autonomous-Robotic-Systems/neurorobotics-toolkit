#ifndef MANAGED_READER_HPP_
#define MANAGED_READER_HPP_

#include "sensors/common/utils.h"

#include "rosbag2_transport/player.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/play_options.hpp"

class ManagedReader : public rclcpp_lifecycle::LifecycleNode
{
	public:
		/**
		 * @brief Construct a new Managed Reader object.
		 * @param options The NodeOptions for this lifecycle node, passed from the component manager.
		 */
		explicit ManagedReader(const rclcpp::NodeOptions & options);

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

		/// @brief A shared pointer to the main rosbag2 player transport object.
		std::shared_ptr<rosbag2_transport::Player> mpPlayer;

		/// @brief A shared pointer to the storage options for the bag file.
		std::shared_ptr<rosbag2_storage::StorageOptions> mpStorageOptions;

		/// @brief A shared pointer to the playback options.
		std::shared_ptr<rosbag2_transport::PlayOptions> mpPlayOptions;

		/// @brief The URI (path) for the input bag file.
		std::string mpInputBagName;

		/// @brief A vector of topic names to be played from the bag.
		std::vector<std::string> mvpTopicsToPublish;
};

#endif // MANAGED_READER_HPP_
