#!/usr/bin/env python3

import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
from deepstream_class import Pipeline 
import pyds

def osd_sink_pad_buffer_probe(pad,info,u_data):
    gst_buffer = info.get_buffer()
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number=frame_meta.frame_num
        num_rects = frame_meta.num_obj_meta
        l_obj=frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            rect_params = obj_meta.rect_params
            top = int(rect_params.top)
            left = int(rect_params.left)
            width = int(rect_params.width)
            height = int(rect_params.height)

            x1 = left
            y1 = top
            x2 = left + width
            y2 = top + height
            print(x1, y1, x2, y2)
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

def main(args):
    pipeline = Pipeline(args[1], args[2])
    #pipeline = Pipeline('sample_720p.h264', 'yolov7-tiny-320.txt')
    pipeline.osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)
    pipeline.run()

if __name__ == '__main__':
    sys.exit(main(sys.argv))

