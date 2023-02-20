#!/usr/bin/env python3

import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
from deepstream_class import pipeline 
import pyds

def main(args):
    Pipeline = pipeline(args[1], args[2])
    Pipeline.run()

if __name__ == '__main__':
    sys.exit(main(sys.argv))