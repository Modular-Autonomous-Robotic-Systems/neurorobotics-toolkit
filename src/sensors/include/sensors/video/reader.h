#include "sensors/video/common.h"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <gst/app/app.h>
#include <thread>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

struct Frame{
    public:
        cv::Mat frame;
        uint64 timestamp;
        std::string format;
        int height, width;
};

class VideoReaderNode;

class VideoReader {
    public:
        VideoReader(std::shared_ptr<VideoReaderNode> nh, const std::string& inputFilePath, std::function<void(std::shared_ptr<Frame>)> cb);
        ~VideoReader();
        void run();
		void startReader();
        void shutdown();
        void frameCallback(std::shared_ptr<Frame> frame);
    private:
		std::shared_ptr<VideoReaderNode> mpNodeHandle;
        rclcpp::Logger mpLogger;
        std::string mpInputFilePath;
        std::function<void(std::shared_ptr<Frame>)> mpFrameCallback;

        GstElement *mpPipeline;
        GstElement *mpFilesrc;
        GstElement *mpDemuxer;
        GstElement *mpQueue;
        GstElement *mpH264Parser;
        GstElement *mpH264Decoder;
        GstElement *mpVideoconvert;
        GstElement *mpCapsfilter;
        GstElement *mpAppsink;
        GMainLoop *loop;

        gboolean bus_call(GstBus *bus, GstMessage *msg);
        static GstFlowReturn new_sample(GstAppSink *sink, gpointer user_data);

		std::shared_ptr<std::thread> mpVideoReaderThread;
};

class VideoReaderNode: public rclcpp_lifecycle::LifecycleNode{
    public:
        VideoReaderNode();
        void frameCallback(std::shared_ptr<Frame> frame);

        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

		void onEOSReceived();
		void triggerDeactivate();

    private:
        std::shared_ptr<VideoReader> mpReader;
        std::string mpInputFilePath;
        std::string mpCameraTopic;

		rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mpFramePub;
		// TODO check if SharedPtr from rclcpp::Client can be used here
		std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> mpClientThisGetState;
		std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> mpClientThisSetState;

		unsigned int getState(std::chrono::seconds time_out = 3s);
		bool changeState(std::uint8_t transition, std::chrono::seconds time_out = 10s);
		std::shared_ptr<std::thread> mpDeactivateTriggerThread;
};
