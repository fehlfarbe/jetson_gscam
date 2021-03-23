//
// Created by kolbe on 22.03.21.
//
#include <pluginlib/class_list_macros.h>
#include "jetson_gscam/JetsonGSCam.h"
#include <sstream>

PLUGINLIB_EXPORT_CLASS(jetson_gscam::JetsonGSCam, nodelet::Nodelet)

namespace jetson_gscam {

    void JetsonGSCam::onInit() {
        NODELET_DEBUG("Initializing Jetson GStreamer nodelet...");
        ros::NodeHandle &nh = getPrivateNodeHandle();

        // read params
        sensor_id = nh.param("sensor", 0);
        width = nh.param("width", 1920);
        height = nh.param("height", 1080);
        fps = nh.param("fps", 30);
        flip = nh.param("flip", 0);
        camera_info = nh.param<std::string>("camera_info_url", "");
        camera_name = nh.param<std::string>("camera_info_url", "camera");

        // setup publisher
        pub_raw = nh.advertise<sensor_msgs::Image>("image", 1);
        pub_compressed = nh.advertise<sensor_msgs::CompressedImage>("compressed", 1);

        // setup pipeline
        if (setupPipeline()) {
            startPipeline();
            // start publisher thread
            pub_thread = std::thread(&JetsonGSCam::publisher, this);
            //loop = g_main_loop_new (NULL, TRUE);
            //g_main_loop_run (loop);
        } else {
            NODELET_ERROR("Cannot setup GStreamer pipeline...exit");
        }
    }

    JetsonGSCam::~JetsonGSCam() {
        stopPipeline();

        if (pipeline) {
            gst_object_unref(bus);
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }

	NODELET_INFO("Stopping capture thread...");
        pub_thread_running = false;
        pub_thread.join();
    }

    bool JetsonGSCam::setupPipeline() {
        // gstreamer initialization
        gst_init(nullptr, nullptr);

        // building pipeline
        std::stringstream gst_cmd;
        gst_cmd << "nvarguscamerasrc sensor_id=" << sensor_id << " ! "
                << "tee name=traw traw. ! queue ! nvtee ! "
                << "nvvidconv flip-method=" << flip << " ! "
                << "video/x-raw(memory:NVMM),width=" << width << ",height=" << height <<",framerate=" << fps << "/1,format=NV12 ! "
                << "nvjpegenc ! "
                << "appsink name=\"jpeg_sink\" emit-signals=true drop=true max-buffers=2 "
                << "traw. ! queue ! nvtee ! nvvidconv flip-method=" << flip << " ! "
                << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps << "/1,format=BGRx ! "
                << "videoconvert ! "
                << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps << "/1,format=BGR ! "
                << "appsink name=\"raw_sink\" emit-signals=true drop=true max-buffers=2";

//        gst_cmd << "videotestsrc ! "
//                << "tee name=traw traw. ! queue ! "
//                << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps << "/1,format=NV12 ! "
//                << "jpegenc ! "
//                << "appsink name=\"jpeg_sink\" emit-signals=true "
//                << "traw. ! queue ! "
//                << "videoconvert ! "
//                << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps << "/1,format=BGR ! "
//                << "appsink name=\"raw_sink\" emit-signals=true ";

        NODELET_INFO_STREAM("Pipeline: " << gst_cmd.str());

        GError *error = nullptr;
        pipeline = gst_parse_launch(
                gst_cmd.str().c_str(),
                &error);

        if(error){
            NODELET_ERROR_STREAM("Cannot construct pipeline: " << error->message);
            return false;
        }

        bus = gst_element_get_bus(pipeline);
//        gst_bus_add_watch(bus, &JetsonGSCam::busCallback, this);

//        GstElement *sink = nullptr;
//        auto pads = gst_pipeline_pad
//        auto pads = pipeline->pads;
//        g_object_get(pipeline, "jpeg_sink", sink);
//        auto sink2 = gst_bin_get_by_name(pipeline, "jpeg_sink");
        sink_jpeg = gst_bin_get_by_name (GST_BIN (pipeline), "jpeg_sink");
        sink_raw = gst_bin_get_by_name (GST_BIN (pipeline), "raw_sink");

        /*
         * bus.add_signal_watch()
        bus.connect("message", self._on_playmer_msg)
        jpegsink = self._pipeline.get_by_name("jpeg")
        jpegsink.connect("new-sample", self.on_buffer_jpeg, jpegsink)
        rawsink = self._pipeline.get_by_name("raw")
        rawsink.connect("new-sample", self.on_buffer_raw, rawsink)
         */

        return true;
    }

    void JetsonGSCam::startPipeline() {
        if (pipeline) {
            // start playing
            NODELET_INFO_STREAM("Starting pipeline");
            gst_element_set_state(pipeline, GST_STATE_PLAYING);
            if (gst_element_get_state (pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
                NODELET_INFO("Failed to go into PLAYING state");
//                gst_element_get
            } else {
                NODELET_INFO("Started pipeline");
            }
        }
    }

    void JetsonGSCam::stopPipeline() {
        if (pipeline) {
            // start playing
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
    }

    gboolean JetsonGSCam::busCallback(GstBus *bus, GstMessage *msg, gpointer p) {
        auto *cam = (JetsonGSCam *) p;
        std::cout << cam->camera_name << std::endl;
        return TRUE;
    }

    void JetsonGSCam::publisher() {
        pub_thread_running = true;
	ros::Rate r(fps);
        uint64_t seq = 0;
        while (pub_thread_running && !ros::isShuttingDown()){
            NODELET_DEBUG_STREAM("Running..." << seq);
            bool got_sample = false;
            std_msgs::Header header;
            header.frame_id = camera_name;
            header.stamp = ros::Time::now();
            header.seq = seq;
            
	    if(sink_raw && pub_raw.getNumSubscribers() > 0){
                GstSample *sample;
                g_signal_emit_by_name (sink_raw, "pull-sample", &sample, NULL);
                if(sample){
                    ROS_DEBUG("Got RAW sample");
                    got_sample = true;
                    GstBuffer *buffer;
                    GstMapInfo info;
                    buffer = gst_sample_get_buffer (sample);
                    gst_buffer_map(buffer, &info, GST_MAP_READ);
		    sensor_msgs::Image msg;
                    msg.header = header;
                    msg.width = width;
                    msg.height = height;
                    msg.step = width * 3;
                    msg.encoding = "bgr8";
                    msg.data = std::vector<uint8_t>(info.data, info.data + info.size);
                    pub_raw.publish(msg);
                    gst_buffer_unmap (buffer, &info);
                    gst_sample_unref(sample);
                }
            }

            if(sink_jpeg && pub_compressed.getNumSubscribers() > 0){
                GstSample *sample;
                g_signal_emit_by_name (sink_jpeg, "pull-sample", &sample, NULL);
                if(sample){
                    ROS_DEBUG("Got JPEG sample");
                    got_sample = true;
                    GstBuffer *buffer;
                    GstMapInfo info;
                    buffer = gst_sample_get_buffer (sample);
                    gst_buffer_map(buffer, &info, GST_MAP_READ);
		    sensor_msgs::CompressedImage msg;
                    msg.header = header;
                    msg.format = "jpeg";
                    msg.data = std::vector<uint8_t>(info.data, info.data + info.size);
                    pub_compressed.publish(msg);
                    gst_buffer_unmap (buffer, &info);
                    gst_sample_unref(sample);
                }
            }
            
	    if(got_sample){
                seq++;
            }

            r.sleep();
        }
    }

}
