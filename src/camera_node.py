#!/usr/bin/env python2
import rospy
import rosbag
import os
import signal
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge, CvBridgeError
import subprocess
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject


class JetsonCameraNode:

    def __init__(self):
        self._sensor = rospy.get_param("~sensor", 0)
        self._width = rospy.get_param("~width", 1920)
        self._height = rospy.get_param("~height", 1080)
        self._fps = rospy.get_param("~fps", 30)
        self._flip = rospy.get_param("~flip", 0)
        self._camera_info_url = rospy.get_param("~camera_info_url", None)
        self._camera_name = rospy.get_param("~name", "camera")

        # not implemented at the moment!!
        # self._display = rospy.get_param("~display", False)
        # self._wx = rospy.get_param("~wx", None)
        # self._wy = rospy.get_param("~wy", None)
        # self._ww = rospy.get_param("~ww", None)
        # self._wh = rospy.get_param("~wh", None)
        #
        # # set window size
        # overlay = ""
        # if all(x is not None for x in (self._wx, self._wy, self._ww, self._wh)):
        #     overlay = "window-x={wx} window-y={wy} window-width={ww} window-height={wh}".format(
        #         wx=self._wx, wy=self._wy, ww=self._ww, wh=self._wh
        #     )

        # cv bridge and camera info
        self._bridge = CvBridge()
        self._camera_info = CameraInfoManager(cname=self._camera_name)
        if self._camera_info_url is not None:
            try:
                if not self._camera_info.setURL(self._camera_info_url):
                    rospy.logwarn("Cannot load camera info: {}".format(self._camera_info_url))
            except KeyError as e:
                rospy.logwarn("KeyError while loading camera info: {}".format(e))
            self._camera_info.loadCameraInfo()

        # publishers
        self._pub_compressed = rospy.Publisher(self._camera_name + "/image/compressed", CompressedImage, queue_size=1, latch=True)
        self._pub_raw = rospy.Publisher(self._camera_name + "/image", Image, queue_size=1, latch=True)
        self._pub_camera_info = rospy.Publisher(self._camera_name + "/camera_info", CameraInfo, queue_size=1, latch=True)

        # GStreamer pipeline
        gst_cmd = "nvarguscamerasrc sensor_id={sensor} ! " \
                  "tee name=traw traw. ! queue ! nvtee ! " \
                  "nvvidconv flip-method={flip} ! " \
                  "video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1,format=NV12 ! " \
                  "nvjpegenc ! " \
                  "appsink name=\"jpeg\" emit-signals=true " \
                  "traw. ! queue ! nvtee ! nvvidconv flip-method={flip} ! " \
                  "video/x-raw,width={width},height={height},framerate={fps}/1,format=BGRx ! " \
                  "videoconvert ! " \
                  "video/x-raw,width={width},height={height},framerate={fps}/1,format=BGR ! " \
                  "appsink name=\"raw\" emit-signals=true "
                  # "t. ! queue ! nvtee ! nvvidconv ! nveglglessink sync=false {overlay}"
        # "t. ! queue ! nvtee ! nvvidconv ! nvoverlaysink sync=false {overlay} -e"
        # "t. ! queue ! nvtee ! nvvidconv ! nvoverlaysink sync=false {overlay}"
        # "video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1,format=BGRx ! " \
        # "appsink name=\"raw\" emit-signals=true "

        cmd_parsed = gst_cmd.format(
            sensor=self._sensor,
            width=self._width,
            height=self._height,
            fps=self._fps,
            flip=self._flip,
            # overlay=overlay
        )

        rospy.loginfo("Starting GStreamer pipeline: {}".format(cmd_parsed))

        Gst.init(None)
        self._pipeline = Gst.parse_launch(cmd_parsed)
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_playmer_msg)
        jpegsink = self._pipeline.get_by_name("jpeg")
        jpegsink.connect("new-sample", self.on_buffer_jpeg, jpegsink)
        rawsink = self._pipeline.get_by_name("raw")
        rawsink.connect("new-sample", self.on_buffer_raw, rawsink)

    def run(self):
        # start pipeline
        self._pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        # stop pipeline
        self._pipeline.set_state(Gst.State.NULL)

    def on_buffer_jpeg(self, sink, data):
        if self._pub_compressed.get_num_connections() > 0:
            # rospy.loginfo("Got buffer from {} of length {}".format(sink, data))
            sample = sink.emit("pull-sample")
            buf = sample.get_buffer()
            caps = sample.get_caps()
            # print(caps.get_structure(0).get_value('format'))
            # print(caps.get_structure(0).get_value('height'))
            # print(caps.get_structure(0).get_value('width'))
            # print(buf.get_size())
            # with open("/tmp/test_{:05d}.jpg".format(self._frame), "w") as f:
            #     f.write(buf.extract_dup(0, buf.get_size()))
            #     self._frame += 1
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buf.extract_dup(0, buf.get_size())
            msg.header.stamp = rospy.Time.now()
            self._pub_compressed.publish(msg)
            self.pub_camera_info()

        return Gst.FlowReturn(0)

    def on_buffer_raw(self, sink, data):
        if self._pub_raw.get_num_connections() > 0:
            sample = sink.emit("pull-sample")
            buf = sample.get_buffer()
            # caps = sample.get_caps()
            # format = caps.get_structure(0).get_value('format')
            # width = caps.get_structure(0).get_value('width')
            # height = caps.get_structure(0).get_value('height')
            # print(width, height, format)
            # print(buf.get_size())
            # rospy.loginfo("Got RAW buffer of length {} (expected: {})".format(buf.get_size(), width*height*3))
            # print(buf.extract_dup(0, buf.get_size()))
            msg = Image()
            msg.data = buf.extract_dup(0, buf.get_size())
            msg.height = self._height
            msg.width = self._width
            msg.encoding = "bgr8"
            msg.step = len(msg.data) // msg.height
            msg.header.stamp = rospy.Time.now()
            self._pub_raw.publish(msg)
            self.pub_camera_info()

        return Gst.FlowReturn(0)

    def pub_camera_info(self):
        if not self._camera_info.isCalibrated():
            rospy.logwarn_once("Camera is not calibrated!")
            return
        cam_info = self._camera_info.getCameraInfo()
        cam_info.header.frame_id = self._camera_name
        self._pub_camera_info.publish(cam_info)

    def _on_playmer_msg(self, bus, msg):
        try:
            rospy.loginfo("{} {} {} {}".format(bus, Gst.MessageType.get_name(msg.type), msg.src.name,
                                               msg.get_structure().to_string()))
        except Exception as e:
            rospy.logerror(e)
        t = msg.type
        if t == Gst.MessageType.EOS:
            self._pipeline.set_state(Gst.State.NULL)
            rospy.loginfo("Pipeline end")
        elif t == Gst.MessageType.ERROR:
            self._pipeline.set_state(Gst.State.NULL)
            err, debug = msg.parse_error()
            rospy.logerr("Pipeline Error {}".format(err))


if __name__ == "__main__":
    rospy.init_node("jetson_gscam")
    cam = JetsonCameraNode()
    try:
        cam.run()
        rospy.spin()
    except rospy.ROSException as e:
        cam.stop()
        rospy.logerr(e)

