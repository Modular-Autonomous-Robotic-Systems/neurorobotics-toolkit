#include <tello_controller/controller.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<TelloControllerNode> node = std::make_shared<TelloControllerNode>("tello_controller_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}