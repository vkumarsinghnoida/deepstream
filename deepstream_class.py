#!/usr/bin/env python3

import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst

import pyds

def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t==Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        sys.stderr.write("Warning: %s: %s\n" % (err, debug))
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        loop.quit()
    return True

class Pipeline:
    def __init__(self, file_path, config_file):
        Gst.init(None)

        self.pipeline = Gst.Pipeline()
        self.source = Gst.ElementFactory.make("filesrc", "file-source")
        self.h264parser = Gst.ElementFactory.make("h264parse", "h264-parser")
        self.decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
        self.streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
        self.pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
        self.nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
        self.nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
        self.sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
        self.transform = Gst.ElementFactory.make("nvegltransform", "nvegl-transform")

        self.source.set_property('location', file_path)
        self.streammux.set_property('width', 1920)
        self.streammux.set_property('height', 1080)
        self.streammux.set_property('batch-size', 1)
        self.streammux.set_property('batched-push-timeout', 4000000)
        self.pgie.set_property('config-file-path', config_file)

        self.pipeline.add(self.source)
        self.pipeline.add(self.h264parser)
        self.pipeline.add(self.decoder)
        self.pipeline.add(self.streammux)
        self.pipeline.add(self.pgie)
        self.pipeline.add(self.nvvidconv)
        self.pipeline.add(self.nvosd)
        self.pipeline.add(self.sink)
        self.pipeline.add(self.transform)

        sinkpad = self.streammux.get_request_pad("sink_0")
        srcpad = self.decoder.get_static_pad("src")

        self.source.link(self.h264parser)
        self.h264parser.link(self.decoder)
        srcpad.link(sinkpad)
        self.streammux.link(self.pgie)
        self.pgie.link(self.nvvidconv)
        self.nvvidconv.link(self.nvosd)
        self.nvosd.link(self.transform)
        self.transform.link(self.sink)

        self.osdsinkpad = self.nvosd.get_static_pad("sink")
        self.osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, self.osd_sink_pad_buffer_probe, 0)

    def osd_sink_pad_buffer_probe(self, pad,info,u_data):
    
        gst_buffer = info.get_buffer()
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
        l_frame = batch_meta.frame_meta_list
        while l_frame is not None:
            try:
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break

            while l_obj is not None:
                try:
                    obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break
                obj_counter[obj_meta.class_id] += 1
                obj_meta.rect_params.border_color.set(0.0, 0.0, 1.0, 0.0)
                try: 
                    l_obj=l_obj.next
                except StopIteration:
                    break

            try:
                l_frame=l_frame.next
            except StopIteration:
                break
			
        return Gst.PadProbeReturn.OK

    def run(self):

        self.pipeline.set_state(Gst.State.PLAYING)

        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect ("message", bus_call, self.loop)

        try:
            self.loop.run()
        except:
            pass

        self.pipeline.set_state(Gst.State.NULL)


def main(args):
    pipeline = pipeline(args[1], args[2])
    pipeline.run()

if __name__ == '__main__':
    sys.exit(main(sys.argv))
