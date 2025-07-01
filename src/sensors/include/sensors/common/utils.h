#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>
#include <vector>
#include <map>

typedef struct {
	int mWidth;
	int mHeight;
	int mNumChannels;
	int mFPS;
} VideoProperties;

void check_element_creation(GstElement *element, const std::string &name);
void check_linking(bool success, const std::string &linking);

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
