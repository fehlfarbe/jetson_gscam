//
// Created by kolbe on 22.03.21.
//

#ifndef JETSON_GSCAM_JETSONGSCAM_H
#define JETSON_GSCAM_JETSONGSCAM_H

#include <cstring>
#include <iostream>
#include <thread>
#include <gst/gst.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>


namespace jetson_gscam {

    class JetsonGSCam : public nodelet::Nodelet {
    public:
        void onInit() override;
        ~JetsonGSCam() override;

        bool setupPipeline();
        void startPipeline();
        void stopPipeline();

        void publisher();

        static gboolean busCallback(GstBus *bus, GstMessage *msg, gpointer p);

    protected:
        uint32_t sensor_id;
        uint32_t width;
        uint32_t height;
        uint32_t fps;
        uint32_t  flip;
        std::string camera_info;
        std::string camera_name;

        // publisher
        bool pub_thread_running = false;
        std::thread pub_thread;
        ros::Publisher pub_raw;
        ros::Publisher pub_compressed;

        //Gst
        GstElement *pipeline = nullptr;
        GstBus *bus = nullptr;
//        GstMessage *msg = nullptr;
        GMainLoop *loop = nullptr;
        GstElement *sink_raw = nullptr;
        GstElement *sink_jpeg = nullptr;
    };
}


#endif //JETSON_GSCAM_JETSONGSCAM_H
