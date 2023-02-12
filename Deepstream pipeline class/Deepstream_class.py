#!/usr/bin/env python3

import gi
import sys
sys.path.append(../)
gi.require_version('Gst', '1.0')
from gi.repository import Gst, Glib
import pyds

class ObjectDetection:
    def __init__(self, stream_url, config_file_path):
        self.stream_url = stream_url
        self.config_file_path = config_file_path

        Gst.init(None)
        self.pipeline = Gst.pipeline()
        
        self.source = Gst.ElementFactory.make("filesrc", "file-source")
        self.h264parser = Gst.ElementFactory.make("h264parse", "h264-parser")
        self.decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
        self.streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
        self.pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
        self.nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
        self.nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
        self.sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
        self.sinkpad = self.streammux.get_request_pad("sink_0")
        self.srcpad = self.decoder.get_static_pad("src")
        
        self.source.set_property('location', self.stream_url)
        self.pgie.set_property('config-file-path', self.config_file_path)
        
        self.pipeline.add(self.
        self.pipeline.add(self.
        self.pipeline.add(self.
        self.pipeline.add(self.
        self.pipeline.add(self.
        self.pipeline.add(self.

