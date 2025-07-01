#include "sensors/common/utils.h"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <gst/app/app.h>
#include <thread>
#include <atomic>

struct Frame{
    public:
        cv::Mat frame;
        double timestamp;
        std::string format;
        int height, width;
};

class VideoReader {
    public:
        VideoReader(rclcpp::Logger logger, const std::string& inputFilePath, std::function<void(std::shared_ptr<Frame>)> cb);
        ~VideoReader();
        void run();
        void shutdown();
        void frameCallback(std::shared_ptr<Frame> frame);
    private:
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

    private:
        std::shared_ptr<VideoReader> mpReader;
        std::string mpInputFilePath;
        std::string mpCameraTopic;

        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mpFramePub;
};
