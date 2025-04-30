#include "sensors/video/reader.h"

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
	auto end = std::chrono::steady_clock::now() + time_to_wait;
	std::chrono::milliseconds wait_period(100);
	std::future_status status = std::future_status::timeout;
	do {
		auto now = std::chrono::steady_clock::now();
		auto time_left = end - now;
		if (time_left <= std::chrono::seconds(0)) {break;}
		status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
	} while (rclcpp::ok() && status != std::future_status::ready);
	return status;
}

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
			RCLCPP_INFO(mpLogger, "End of Stream");
            g_main_loop_quit(loop);
			mpNodeHandle->onEOSReceived();
            return G_SOURCE_REMOVE;
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(msg, &err, &debug_info);
			RCLCPP_ERROR(mpLogger, "Error: %s", err->message);
			RCLCPP_ERROR(mpLogger, "Debug Info: %s", (debug_info ? debug_info : "none"));
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

void VideoReader::startReader(){
    loop = g_main_loop_new(nullptr, FALSE);

    GstBus *bus = gst_element_get_bus(mpPipeline);
    gst_bus_add_watch(bus, +[](GstBus *bus, GstMessage *msg, gpointer data) {
        return static_cast<VideoReader*>(data)->bus_call(bus, msg);
    }, this);

    check_linking(gst_element_set_state(mpPipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE, "pipeline state to PLAYING");

    g_main_loop_run(loop);

    gst_object_unref(bus);
	
}

void VideoReader::run() {
	RCLCPP_INFO(mpLogger, "Spawning Video Reader thread");
	mpVideoReaderThread = std::make_shared<std::thread>(&VideoReader::startReader, this);
	rclcpp::sleep_for(std::chrono::milliseconds(2));
}

void VideoReader::shutdown() {
	RCLCPP_INFO(mpLogger, "Shutting Down Video Reader");
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
	if(mpVideoReaderThread->joinable()){
		mpVideoReaderThread->join();
	}
	RCLCPP_INFO(mpLogger, "Cleanup of Video Reader Done.");
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
	mpClientThisGetState = this->create_client<lifecycle_msgs::srv::GetState>("video_reader/get_state");
	mpClientThisSetState = this->create_client<lifecycle_msgs::srv::ChangeState>("video_reader/change_state");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoReaderNode::on_activate(const rclcpp_lifecycle::State &){
	mpFramePub->on_activate();
	std::function<void(std::shared_ptr<Frame>)> fcb = [this](std::shared_ptr<Frame> frame){
		frameCallback(frame);
	};
	std::shared_ptr<VideoReaderNode> this_shrd = std::shared_ptr<VideoReaderNode>(this);
	mpReader = std::make_shared<VideoReader>(this_shrd, mpInputFilePath, fcb);
	RCLCPP_INFO(this->get_logger(), "Starting Video Reader");
	mpReader->run();
	RCLCPP_INFO(this->get_logger(), "Spawned Video Reader Node");
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
    RCLCPP_INFO(this->get_logger(), "Video reader node cleanup started");
    // mpReader.reset();
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


void VideoReaderNode::triggerDeactivate(){
	rclcpp::sleep_for(std::chrono::milliseconds(5));
	this->changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void VideoReaderNode::onEOSReceived(){
	mpDeactivateTriggerThread = std::make_shared<std::thread>(&VideoReaderNode::triggerDeactivate, this);
}

/// Requests the current state of the current node
/**
 * In this function, we send a service request
 * asking for the current state of the node
 * lc_talker.
 * If it does return within the given time_out,
 * we return the current state of the node, if
 * not, we return an unknown state.
 * \param time_out Duration in seconds specifying
 * how long we wait for a response before returning
 * unknown state
 */
unsigned int VideoReaderNode::getState(std::chrono::seconds time_out)
{
	auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

	if (!mpClientThisGetState->wait_for_service(time_out)) {
		RCLCPP_ERROR(this->get_logger(), "Service %s is not available.", mpClientThisGetState->get_service_name());
		return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}

	// We send the service request for asking the current
	// state of the lc_talker node.
	auto future_result = mpClientThisGetState->async_send_request(request).future.share();

	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
		RCLCPP_ERROR(this->get_logger(), "Server time out while getting current state for node video_reader");
		return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}

	// We have an succesful answer. So let's print the current state.
	if (future_result.get()) {
		RCLCPP_INFO(
				get_logger(), "Node %s has current state %s.",
				"video_reader", future_result.get()->current_state.label.c_str());
		return future_result.get()->current_state.id;
	} else {
		RCLCPP_ERROR(
				get_logger(), "Failed to get current state for node %s", "video_reader");
		return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}
}

/// Invokes a transition of the current node
/**
 * We send a Service request and indicate
 * that we want to invoke transition with
 * the id "transition".
 * By default, these transitions are
 * - configure
 * - activate
 * - cleanup
 * - shutdown
 * \param transition id specifying which
 * transition to invoke
 * \param time_out Duration in seconds specifying
 * how long we wait for a response before returning
 * unknown state
 */
bool VideoReaderNode::changeState(std::uint8_t transition, std::chrono::seconds time_out)
{
	auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
	request->transition.id = transition;

	if (!mpClientThisSetState->wait_for_service(time_out)) {
		RCLCPP_ERROR(
				get_logger(),
				"Service %s is not available.",
				mpClientThisSetState->get_service_name());
		return false;
	}

	// We send the request with the transition we want to invoke.
	auto future_result = mpClientThisSetState->async_send_request(request).future.share();

	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
		RCLCPP_ERROR(
				get_logger(), "Server time out while getting changing state for node video_reader");
		return false;
	}

	// We have an answer, let's print our success.
	if (future_result.get()->success) {
		RCLCPP_INFO(
				get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
		return true;
	} else {
		RCLCPP_WARN(
				get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
		return false;
	}
}
