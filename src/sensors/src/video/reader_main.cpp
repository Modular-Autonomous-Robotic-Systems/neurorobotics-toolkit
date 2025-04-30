#include "sensors/video/reader.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoReaderNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node->get_node_base_interface());

	executor.spin();

    rclcpp::shutdown();
    return 0;
}
