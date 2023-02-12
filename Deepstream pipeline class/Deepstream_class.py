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

