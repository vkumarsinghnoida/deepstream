#!/usr/bin/env python3

import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
from deepstream_class import Pipeline 
import pyds

def main(args):
    pipeline = Pipeline(args[1], args[2])
    pipeline.osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)
    pipeline.run()

if __name__ == '__main__':
    sys.exit(main(sys.argv))
