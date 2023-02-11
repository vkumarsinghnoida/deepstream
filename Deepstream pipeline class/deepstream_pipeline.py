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
        self.pipeline = Gst.Pipeline.new("object-detection-pipeline")

        # Create the source element to read the stream
        self.src = Gst.ElementFactory.make("rtspsrc", "stream-source")
        self.src.set_property("location", self.stream_url)
        self.pipeline.add(self.src)

        # Create a deepstream decoder to decode the video stream
        self.decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
        self.pipeline.add(self.decoder)

        # Link the source and decoder elements
        self.src.link(self.decoder)

        # Create a deepstream inference element to perform object detection
        self.nvinfer = Gst.ElementFactory.make("nvinfer", "nvinfer")
        self.pipeline.add(self.nvinfer)

        # Set the inference configuration
        self.nvinfer.set_property("config-file-path", self.config_file_path)

        # Create a deepstream object-detection element to perform object detection
        self.osd = Gst.ElementFactory.make("nvdsosd", "nvosd")
        self.pipeline.add(self.osd)

        # Link the pipeline elements
        self.decoder.link(self.nvinfer)
        self.nvinfer.link(self.osd)

        # Create a deepstream render element to display the detections
        self.sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
        self.pipeline.add(self.sink)

        # Link the render element to the object detection element
        self.osd.link(self.sink)

    def start(self):
        # Use the osd_sink_pad_buffer_probe function and the pyds library to extract the metadata of the object detections
        def osd_sink_pad_buffer_probe(pad, info, u_data):
            sample = info.get_buffer()
            if sample:
                # Get the object detection metadata from the sample using the pyds library
                obj_meta = pyds.NvDsObjectMeta.cast(sample.get_nvds_meta(0))
                while obj_meta:
                    # Display the detections
                    # Code to display the detections goes here
                    obj_meta = obj_meta.next
            return Gst.PadProbeReturn.OK

        osd_sink_pad = self.osd.get_static_pad("sink")
        osd_sink_pad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, None)

        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        # Run the GstMainLoop to keep the pipeline running
        self.main_loop = Gst.MainLoop.new(None, False)
        self.main_loop.run()
        self.pipeline.set_state(Gst.State.NULL)
        # Quit the GstMainLoop
        self.main_loop.quit()

# Example usage:
if __name__ == '__main__':
    stream_url = "rtsp://<stream_url>"
    config_file_path = "/path/to/config_file.txt"
    obj_detection = ObjectDetection(stream_url, config_file_path)
    obj_detection.start()
