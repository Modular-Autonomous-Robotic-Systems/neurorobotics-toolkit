#include "sensors/video/logger.h"

// TODO move the main method in to `src/video/logger.cpp`
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoLoggerNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    executor.spin();
    
	rclcpp::shutdown();
    return 0;
}
