#include "sensors/video/reader.h"

VideoReader::VideoReader(std::shared_ptr<VideoReaderNode> nh, const std::string& mpInputFilePath, std::function<void(std::shared_ptr<Frame>)> cb):
mpNodeHandle(nh), mpLogger(nh->get_logger()), mpInputFilePath(mpInputFilePath), mpFrameCallback(cb), mpPipeline(nullptr),
mpFilesrc(nullptr), mpDemuxer(nullptr), mpQueue(nullptr), mpH264Parser(nullptr),
mpH264Decoder(nullptr), mpVideoconvert(nullptr), mpCapsfilter(nullptr), mpAppsink(nullptr),
loop(nullptr) {

    gst_init(nullptr, nullptr);

    mpPipeline = gst_pipeline_new("video-reader-pipeline");
    mpFilesrc = gst_element_factory_make("filesrc", "source");
    mpDemuxer = gst_element_factory_make("qtdemux", "demuxer");
    mpQueue = gst_element_factory_make("queue", "queue");
    mpH264Parser = gst_element_factory_make("h264parse", "parser");
    mpH264Decoder = gst_element_factory_make("avdec_h264", "decoder");
    mpVideoconvert = gst_element_factory_make("videoconvert", "converter");
    mpCapsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    mpAppsink = gst_element_factory_make("appsink", "sink");

    check_element_creation(mpPipeline, "pipeline");
    check_element_creation(mpFilesrc, "filesrc");
    check_element_creation(mpDemuxer, "qtdemux");
    check_element_creation(mpQueue, "queue");
    check_element_creation(mpH264Parser, "h264parse");
    check_element_creation(mpH264Decoder, "avdec_h264");
    check_element_creation(mpVideoconvert, "videoconvert");
    check_element_creation(mpCapsfilter, "capsfilter");
	// TODO add the following video rate limiter:
	// https://stackoverflow.com/a/42252285/9090538
    check_element_creation(mpAppsink, "appsink");

    g_object_set(mpFilesrc, "location", mpInputFilePath.c_str(), NULL);
    g_object_set(mpAppsink, "emit-signals", TRUE, "sync", TRUE, "max-lateness", 50 * GST_MSECOND, "qos", TRUE, NULL);

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        NULL);
    g_object_set(mpCapsfilter, "caps", caps, NULL);
    gst_caps_unref(caps);

    gst_bin_add_many(GST_BIN(mpPipeline), mpFilesrc, mpDemuxer, mpQueue, mpH264Parser, mpH264Decoder, mpVideoconvert, mpCapsfilter, mpAppsink, NULL);

    check_linking(gst_element_link(mpFilesrc, mpDemuxer), "source to demuxer");
    check_linking(gst_element_link_many(mpQueue, mpH264Parser, mpH264Decoder, mpVideoconvert, mpCapsfilter, mpAppsink, NULL), "linking elements");

    g_signal_connect(mpDemuxer, "pad-added", G_CALLBACK(+[](GstElement *src, GstPad *pad, gpointer data) {
        GstElement *mpQueue = (GstElement *)data;
        GstPad *sink_pad = gst_element_get_static_pad(mpQueue, "sink");
        if (gst_pad_link(pad, sink_pad) != GST_PAD_LINK_OK) {
            g_print("Failed to link demuxer pad to queue.\n");
        }
        gst_object_unref(sink_pad);
    }), mpQueue);

    g_object_set(mpQueue,
        "max-size-buffers", 2,   
        "max-size-time", 0,    
        "max-size-bytes", 0,    
        NULL);

    g_object_set(mpH264Decoder,
        "max-threads", 4,
        NULL);

    g_signal_connect(mpAppsink, "new-sample", G_CALLBACK(new_sample), this);
}

VideoReader::~VideoReader() {
    if (mpPipeline) {
        gst_element_set_state(mpPipeline, GST_STATE_NULL);
        gst_object_unref(mpPipeline);
    }

    if (loop) {
        if (g_main_loop_is_running(loop)) {
            g_main_loop_quit(loop);
        }
        g_main_loop_unref(loop);
    }
}

void VideoReader::frameCallback(std::shared_ptr<Frame> frame){
	this->mpFrameCallback(frame);
}

GstFlowReturn VideoReader::new_sample(GstAppSink *sink, gpointer user_data) {
    VideoReader *reader = static_cast<VideoReader*>(user_data);
    
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
        RCLCPP_WARN(reader->mpLogger, "Failed to get new sample, pipeline may be stopping");
        return GST_FLOW_OK;
    }

    try {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (buffer) {
            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                GstClockTime timestamp = GST_BUFFER_PTS(buffer);

                cv::Mat frame(cv::Size(1920, 1080), CV_8UC3, (void*)map.data, cv::Mat::AUTO_STEP);
                std::shared_ptr<Frame> f = std::make_shared<Frame>();
				if(!frame.empty()){
					f->frame = frame.clone();
					f->timestamp = timestamp;
					reader->frameCallback(f);
				}
				else{
					RCLCPP_ERROR(reader->mpLogger, "Received empty frame");
				}

                gst_buffer_unmap(buffer, &map);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(reader->mpLogger, "Error processing frame: %s", e.what());
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

gboolean VideoReader::bus_call(GstBus *bus, GstMessage *msg) {
    switch (msg->type) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            return G_SOURCE_REMOVE;
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(msg, &err, &debug_info);
            std::cerr << "Error: " << err->message << std::endl;
            std::cerr << "Debug Info: " << (debug_info ? debug_info : "none") << std::endl;
            g_clear_error(&err);
            g_free(debug_info);
            g_main_loop_quit(loop);
            return G_SOURCE_REMOVE;
        }
        default:
            return G_SOURCE_CONTINUE;
    }
    return G_SOURCE_CONTINUE;
}

void VideoReader::run() {
    loop = g_main_loop_new(nullptr, FALSE);

    GstBus *bus = gst_element_get_bus(mpPipeline);
    gst_bus_add_watch(bus, +[](GstBus *bus, GstMessage *msg, gpointer data) {
        return static_cast<VideoReader*>(data)->bus_call(bus, msg);
    }, this);

    check_linking(gst_element_set_state(mpPipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE, "pipeline state to PLAYING");

    g_main_loop_run(loop);

    gst_object_unref(bus);
}

void VideoReader::shutdown() {
    if (loop) {
        g_main_loop_quit(loop); 
    }
    if (mpPipeline) {
        gst_element_set_state(mpPipeline, GST_STATE_NULL);
        gst_object_unref(mpPipeline);
        mpPipeline = nullptr;
    }
    if (loop) {
        g_main_loop_unref(loop);
        loop = nullptr;
    }
}

VideoReaderNode::VideoReaderNode() : rclcpp_lifecycle::LifecycleNode("video_reader"){
	ParameterDescriptor topicNameDesc;
	topicNameDesc.description = "Name of topic to publish to";
	topicNameDesc.type = 4;
	this->declare_parameter<std::string>("camera_topic", "/camera", topicNameDesc);

	ParameterDescriptor inputFilePathDesc;
	inputFilePathDesc.description = "Path to input file ";
	inputFilePathDesc.type = 4;
	this->declare_parameter<std::string>("input_file_path", "/ws/data/src.mp4", inputFilePathDesc);
}

CallbackReturn VideoReaderNode::on_configure(const rclcpp_lifecycle::State &){
    mpInputFilePath = this->get_parameter("input_file_path").as_string();
    mpCameraTopic = this->get_parameter("camera_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Reading file from %s", mpInputFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic for publication: %s", mpCameraTopic.c_str());

    mpFramePub =  this->create_publisher<sensor_msgs::msg::Image>(mpCameraTopic, 10);
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoReaderNode::on_activate(const rclcpp_lifecycle::State &){
    if (mpFramePub) {
        mpFramePub->on_activate();
    }
    if (!mpReader){
        std::function<void(std::shared_ptr<Frame>)> fcb = [this](std::shared_ptr<Frame> frame){
            frameCallback(frame);
        };
		std::shared_ptr<VideoReaderNode> this_shrd = std::shared_ptr<VideoReaderNode>(this);
        mpReader = std::make_shared<VideoReader>(this_shrd, mpInputFilePath, fcb);
        RCLCPP_INFO(this->get_logger(), "Running the pipeline...");
        mpReader->run();
        RCLCPP_INFO(this->get_logger(), "Pipeline stopped");
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoReaderNode::on_deactivate(const rclcpp_lifecycle::State &){
    if (mpReader){
        mpReader->shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Video reader node deactivated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoReaderNode::on_cleanup(const rclcpp_lifecycle::State &){
    mpReader.reset();
    RCLCPP_INFO(this->get_logger(), "Video reader node cleaned.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoReaderNode::on_shutdown(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(this->get_logger(), "Video reader node shut down.");
    return CallbackReturn::SUCCESS;
}

void VideoReaderNode::frameCallback(std::shared_ptr<Frame> frame) {
    static rclcpp::Time last_pub_time = this->now();
    rclcpp::Time current_time = this->now();

    double target_period = 1.0/30.0;
    if ((current_time - last_pub_time).seconds() >= target_period) {
        if(!frame->frame.empty()) {
            sensor_msgs::msg::Image img_msg;
            std_msgs::msg::Header header;
            header.stamp = rclcpp::Time(frame->timestamp);

            cv_bridge::CvImage(
                header,
                sensor_msgs::image_encodings::RGB8,
                frame->frame).toImageMsg(img_msg);

            mpFramePub->publish(img_msg);
            last_pub_time = current_time;
        }
    }
}
