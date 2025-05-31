#include "sensors/video/logger.h"

VideoLogger::VideoLogger(rclcpp::Logger logger, std::string &outputPath, VideoProperties &props):
mpLogger(logger){ 
    mpOutputFilePath = outputPath;
    mpOutputImageWidth = props.mWidth;
    mpOutputImageHeight = props.mHeight;
    mpInputDataFPS = props.mFPS;
    mpNumChannels = props.mNumChannels;
    mpOutputFrameSize = mpOutputImageWidth * mpOutputImageHeight * mpNumChannels;
    mpFrameCounter = 0;
    
    gst_init(nullptr, nullptr);

    mpPipeline = gst_pipeline_new("video-logger-pipeline");
    mpAppsrc = gst_element_factory_make("appsrc", "mysource");
    mpVideoconvert = gst_element_factory_make("videoconvert", "myconvert");
    mpX264enc = gst_element_factory_make("x264enc", "myencoder");
	mpH264Parser = gst_element_factory_make("h264parse", "parser");
    mpMp4mux = gst_element_factory_make("mp4mux", "mymux");
    mpFilesink = gst_element_factory_make("filesink", "myfileoutput");

    check_element_creation(mpPipeline, "pipeline");
    check_element_creation(mpAppsrc, "appsrc");
    check_element_creation(mpVideoconvert, "videoconvert");
    check_element_creation(mpX264enc, "x264encoder");
	check_element_creation(mpH264Parser, "parser");
    check_element_creation(mpMp4mux, "mp4muxer");
    check_element_creation(mpFilesink, "filesink");

    if (!mpPipeline || !mpAppsrc || !mpVideoconvert || !mpX264enc || !mpH264Parser || !mpMp4mux || !mpFilesink) {
        RCLCPP_ERROR(mpLogger, "Not all elements could be created.");
        mpVideoLoggerStatus = -1;
        return;
    }
    
    // Configure appsrc
    g_object_set(G_OBJECT(mpAppsrc),
        "stream-type", 0,  // GST_APP_STREAM_TYPE_STREAM
        "format", GST_FORMAT_TIME,
        "is-live", TRUE,
        "do-timestamp", TRUE,
        NULL);

    // Configure encoder for better quality and performance
    g_object_set(G_OBJECT(mpX264enc),
        "tune", 0x00000004,  // zerolatency
        "speed-preset", 1,   // superfast
        "bitrate", 2000,     // 2Mbps
        "key-int-max", 30,   // Keyframe every 30 frames
        NULL);

	g_object_set(G_OBJECT(mpH264Parser),
		"config-interval", 1,
		NULL);

    // Set output file
    g_object_set(mpFilesink, "location", mpOutputFilePath.c_str(), NULL);

    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(mpPipeline), mpAppsrc, mpVideoconvert, mpX264enc, mpH264Parser, mpMp4mux, mpFilesink, NULL);
    
    if (!gst_element_link_many(mpAppsrc, mpVideoconvert, mpX264enc, mpH264Parser, mpMp4mux, mpFilesink, NULL)) {
        RCLCPP_ERROR(mpLogger, "Elements could not be linked");
        mpVideoLoggerStatus = -1;
        return;
    }

    // Set caps for BGR format
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, mpOutputImageWidth,
        "height", G_TYPE_INT, mpOutputImageHeight,
        "framerate", GST_TYPE_FRACTION, mpInputDataFPS, 1,
        NULL);

    if (!caps) {
        RCLCPP_ERROR(mpLogger, "Failed to create caps");
        mpVideoLoggerStatus = -1;
        return;
    }
    
    gst_app_src_set_caps(GST_APP_SRC(mpAppsrc), caps);
    gst_caps_unref(caps);

    mpVideoLoggerStatus = 1;
}

// Set the pipeline to playing state to start receivving frames
bool VideoLogger::start() {
    GstStateChangeReturn ret = gst_element_set_state(mpPipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    return true;
}

void VideoLogger::stop() {
    RCLCPP_INFO(mpLogger, "Starting video logger shutdown sequence");

    try {
        cv::destroyWindow("Video Logger Output");
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        RCLCPP_WARN(mpLogger, "Error closing OpenCV window: %s", e.what());
    }

    // Send EOS to the pipeline
    if (mpAppsrc) {
        RCLCPP_INFO(mpLogger, "Sending EOS to pipeline");
        GstFlowReturn ret = gst_app_src_end_of_stream(GST_APP_SRC(mpAppsrc));
        if (ret != GST_FLOW_OK) {
            RCLCPP_ERROR(mpLogger, "Failed to send EOS");
        }
    }

    // Wait for EOS or error message with timeout
    if (mpPipeline) {
        GstBus *bus = gst_element_get_bus(mpPipeline);
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, 3 * GST_SECOND, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (msg != NULL) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_EOS:
                    RCLCPP_INFO(mpLogger, "Received EOS from pipeline");
                    break;
                case GST_MESSAGE_ERROR:
                    GError *err;
                    gchar *debug;
                    gst_message_parse_error(msg, &err, &debug);
                    RCLCPP_ERROR(mpLogger, "Pipeline error: %s", err->message);
                    g_error_free(err);
                    g_free(debug);
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
        } else {
            RCLCPP_WARN(mpLogger, "Timeout waiting for pipeline EOS");
        }
        gst_object_unref(bus);
    }

    // Stop the pipeline
    if (mpPipeline) {
        RCLCPP_INFO(mpLogger, "Setting pipeline to NULL state");
        GstStateChangeReturn ret = gst_element_set_state(mpPipeline, GST_STATE_NULL);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(mpLogger, "Failed to set pipeline to NULL state");
        } else {
            // Wait for state change with timeout
            GstState state;
            ret = gst_element_get_state(mpPipeline, &state, nullptr, 2 * GST_SECOND);
            if (ret == GST_STATE_CHANGE_FAILURE) {
                RCLCPP_ERROR(mpLogger, "Failed to confirm pipeline state change");
            } else {
                RCLCPP_INFO(mpLogger, "Pipeline successfully stopped");
            }
        }
    }
    
    RCLCPP_INFO(mpLogger, "Video logger shutdown sequence completed");
}

VideoLogger::~VideoLogger() {
    gst_app_src_end_of_stream(GST_APP_SRC(mpAppsrc));

    GstBus *bus = gst_element_get_bus(mpPipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_SECOND * 10,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (msg != NULL) {
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
    gst_element_set_state(mpPipeline, GST_STATE_NULL);
    gst_object_unref(mpPipeline);
	
	cv::destroyWindow("Video Logger Output");
}

// TODO: Save the video file without mismatched timings
void VideoLogger::logFrame(cv::Mat &image) {
    if (image.empty()) {
        RCLCPP_WARN(mpLogger, "Received empty image frame");
        return;
    }

    size_t buf_size = image.total() * image.elemSize();

    // Create buffer
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, buf_size, nullptr);
    if (!buf) {
        RCLCPP_ERROR(mpLogger, "Failed to allocate buffer");
        return;
    }

    // Map buffer for writing
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        RCLCPP_ERROR(mpLogger, "Failed to map buffer");
        gst_buffer_unref(buf); 
        return;
    }

    // Copy frame data
    memcpy(map.data, image.data, buf_size);
    gst_buffer_unmap(buf, &map);

    // Set accurate buffer timestamp
    GstClockTime time = gst_util_uint64_scale_int(mpFrameCounter, GST_SECOND, mpInputDataFPS);
    GST_BUFFER_PTS(buf) = time;
    GST_BUFFER_DTS(buf) = time;
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, mpInputDataFPS);
    
    // Push buffer to pipeline
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(mpAppsrc), buf);
    if (ret != GST_FLOW_OK) {
        RCLCPP_ERROR(mpLogger, "Failed to push buffer: %s", gst_flow_get_name(ret));
        if (ret == GST_FLOW_FLUSHING) {
            RCLCPP_ERROR(mpLogger, "Pipeline is flushing, might be in wrong state");
        }
        return;
    }

    mpFrameCounter++;
}

VideoLoggerNode::VideoLoggerNode() : rclcpp_lifecycle::LifecycleNode("video_logger"){

	ParameterDescriptor outputFilePathDesc;
	outputFilePathDesc.description = "Path to output file for received video";
	outputFilePathDesc.type = 4;
	this->declare_parameter<std::string>("output_file_path", "/ws/data/output.mp4", outputFilePathDesc);

	ParameterDescriptor widthDesc;
	widthDesc.description = "Width of output image";
	widthDesc.type = 2;
	this->declare_parameter<int>("width", 640, widthDesc);

	ParameterDescriptor heightDesc;
	heightDesc.description = "Height of output image";
	heightDesc.type = 2;
	this->declare_parameter<int>("height", 480, heightDesc);

	ParameterDescriptor fpsDesc;
	fpsDesc.description = "FPS of input image stream";
	fpsDesc.type = 2;
	this->declare_parameter<int>("fps", 30, fpsDesc);

	ParameterDescriptor topicNameDesc;
	topicNameDesc.description = "Name of camera topic to subscribe to";
	topicNameDesc.type = 4;
	this->declare_parameter<std::string>("camera_topic", "/camera", topicNameDesc);

	ParameterDescriptor imageTypeDesc;
	imageTypeDesc.description = "Type of Image received by logger";
	imageTypeDesc.type = 4;
	this->declare_parameter<std::string>("image_type", "bgr", imageTypeDesc);

}

CallbackReturn VideoLoggerNode::on_configure(const rclcpp_lifecycle::State &){
	mpOutputFilePath = this->get_parameter("output_file_path").as_string();
	mpOutputImageWidth = this->get_parameter("width").as_int();
	mpOutputImageHeight = this->get_parameter("height").as_int();
	mpInputDataFPS = this->get_parameter("fps").as_int();
	mpTopicName = this->get_parameter("camera_topic").as_string();
	mpImageType = this->get_parameter("image_type").as_string();
    if(mpImageType.compare("bgr") == 0){
        mpNumChannels = 3;    
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image type: %s", mpImageType.c_str());
        return CallbackReturn::ERROR;
    }

	VideoProperties props;
	props.mWidth = mpOutputImageWidth;
	props.mHeight = mpOutputImageHeight;
	props.mNumChannels = mpNumChannels;
	props.mFPS = mpInputDataFPS;

	mpVideoLogger = std::make_unique<VideoLogger>(this->get_logger(), mpOutputFilePath, props);

	mpFrameSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
		mpTopicName.c_str(),
		10,
		std::bind(&VideoLoggerNode::frameCallback, this, std::placeholders::_1)
	);

    mpLifecycleSubscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/video_reader/transition_event", 10,
        std::bind(&VideoLoggerNode::lifecycleCallback, this, std::placeholders::_1)
    );

	RCLCPP_INFO(this->get_logger(), "Created Video Logger Node");
	return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Starting Video Logger Node");
    
    if (!mpVideoLogger) {
        RCLCPP_ERROR(this->get_logger(), "Video Logger instance not initialized");
        return CallbackReturn::ERROR;
    }
    
    if (!mpVideoLogger->start()) {
        RCLCPP_ERROR(this->get_logger(), "Could not start Video Logger Node");
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_deactivate(const rclcpp_lifecycle::State &){
    try {
        RCLCPP_INFO(this->get_logger(), "Deactivating video logger node");
        if (mpFrameSubscriber) {
            mpFrameSubscriber.reset();
        }
        if (mpVideoLogger) {
            mpVideoLogger->stop();
        }
        RCLCPP_INFO(this->get_logger(), "Video logger node deactivated");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during deactivation: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

CallbackReturn VideoLoggerNode::on_cleanup(const rclcpp_lifecycle::State &){
    try {
        if (mpVideoLogger) {
            mpVideoLogger.reset();
        }
        if (mpFrameSubscriber) {
            mpFrameSubscriber.reset();
        }
        if (mpLifecycleSubscriber) {
            mpLifecycleSubscriber.reset();
        }
        RCLCPP_INFO(this->get_logger(), "Video logger node cleaned up");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during cleanup: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

CallbackReturn VideoLoggerNode::on_shutdown(const rclcpp_lifecycle::State &){
    // Nothing to do ???
    RCLCPP_INFO(this->get_logger(), "Video logger shut down");
	return CallbackReturn::SUCCESS;
}

void VideoLoggerNode::frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img) {
    try {
        cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(img, img->encoding);
        
        if (!mpVideoLogger) {
            RCLCPP_ERROR(this->get_logger(), "Video logger not initialized");
            return;
        }

        if (!cv_ptr_img->image.empty()) {
            cv::Mat image;
            if (cv_ptr_img->image.rows == mpOutputImageHeight && 
                cv_ptr_img->image.cols == mpOutputImageWidth) {
                image = cv_ptr_img->image.clone();
            } else {
                cv::resize(cv_ptr_img->image, image, 
                          cv::Size(mpOutputImageWidth, mpOutputImageHeight));
            }
            mpVideoLogger->logFrame(image);
        }
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV error in callback: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in callback: %s", e.what());
    }
}

void VideoLoggerNode::lifecycleCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
    const auto goal_state = msg->goal_state.id;
    if (goal_state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
        RCLCPP_INFO(this->get_logger(), "Video reader has finalized, following suit");
        
        // Use the lifecycle node's built-in state management
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            this->deactivate();
            this->cleanup();
            this->shutdown();
            rclcpp::shutdown();
        }
    }
}
