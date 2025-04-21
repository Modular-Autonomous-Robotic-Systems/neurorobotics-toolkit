
#include <rclcpp/rclcpp.hpp>
#include <tello_msgs/srv/tello_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tello_msgs/msg/flight_data.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <string>
#include <chrono>
#include "lifecycle_msgs/srv/change_state.hpp"



class TelloControllerNode: public rclcpp::Node{
        public:
                TelloControllerNode(std::string nodeName);

                void FlightDataCallback(const tello_msgs::msg::FlightData::ConstSharedPtr &flightData);

                void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img);

                void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo);

                void ActionHandler(const std::shared_ptr<tello_msgs::srv::TelloAction::Request> &request,const std::shared_ptr<tello_msgs::srv::TelloAction::Response> &response);
                
                void ActionRequestSender(const std::string &cmd);

                void Response(const std_msgs::msg::String::ConstSharedPtr &response);

                void TelloStateCallback(const std_msgs::msg::String::ConstSharedPtr &stateMessage);

                void UpdateCurrentTime();

                void changeTeleopState(const std::string state);

                void timer_callback();
        private:
                

                std::string mpTelloFlightState = "landed"; 

                //flags

                bool mpReadyTakeOff = false;

                bool mplowBattery = false;

                //frequency off data

                double mpImageFreq = 0.0;
                
                double mpCameraInfoFreq = 0.0;

                std::string mpActionRequest;

                //client and subscriber
                rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr mpActionClient;

                rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr mpFlightDataSubsriber;

                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpImageRawSubscriber;

                rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mpCameraInforSubscriber;

                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpActionResponseSubscriber;

                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpTelloStateSubscriber;
                
                //Controller lifecycle state client
                rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr mpTeleopLifecycleClient;

                //ros2 timer
                rclcpp::TimerBase::SharedPtr mpFlightChecker;

                rclcpp::Clock::SharedPtr mpClock;

                int32_t mpLastImageTime = 0;

                int32_t mpLastCamInfoTime = 0;
                
 
                
};