#include "sensors/video/reader.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoReaderNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;

    // TODO: add executor and handle lifecycle controlled callback
    // node->configure();
    // node->activate();
    // node->deactivate();
    // node->cleanup();
    // node->shutdown();

    rclcpp::shutdown();
    return 0;
}
