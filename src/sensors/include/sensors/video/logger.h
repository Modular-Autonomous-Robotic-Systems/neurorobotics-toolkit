#include "sensors/video/common.h"
#include <gst/app/gstappsrc.h>

class VideoLoggerNode;

class VideoLogger{
	public:
		VideoLogger(std::shared_ptr<VideoLoggerNode> nh, std::string &outputPath, VideoProperties &props);
		~VideoLogger();
		bool start();
		void stop();
		int getStatus(){return mpVideoLoggerStatus;}
		void logFrame(cv::Mat &image, uint64 timestamp);

	private:
		rclcpp::Logger mpLogger;
		std::shared_ptr<VideoLoggerNode> mpNodeHandle;

		GstElement* mpPipeline;
		GstElement* mpAppsrc;
		GstElement* mpVideoconvert;
		GstElement* mpX264enc;
		GstElement* mpH264Parser;
		GstElement* mpMp4mux;
		GstElement* mpFilesink;
		GstMapInfo mpMap;
		
		std::string mpOutputFilePath;
		int mpOutputImageWidth;
		int mpOutputImageHeight;
		int mpNumChannels;
		int mpInputDataFPS;
		int mpOutputFrameSize;
		int mpVideoLoggerStatus = 0;
		int mpFrameCounter;

};

class VideoLoggerNode: public rclcpp_lifecycle::LifecycleNode{
	public:
		VideoLoggerNode();
		void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img);
		void lifecycleCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

		CallbackReturn on_configure(const rclcpp_lifecycle::State &);
		CallbackReturn on_activate(const rclcpp_lifecycle::State &);
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
		CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

	private:
		std::string mpOutputFilePath;
		std::string mpTopicName;
		std::string mpImageType;

		int mpOutputImageWidth;
		int mpOutputImageHeight;
		int mpInputDataFPS;
		int mpNumChannels;

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;
		rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr mpLifecycleSubscriber;

		std::unique_ptr<VideoLogger> mpVideoLogger;
};
