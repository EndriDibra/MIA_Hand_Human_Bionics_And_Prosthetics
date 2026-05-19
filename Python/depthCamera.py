# Author: Endri Dibra 

# Project: Intel RealSense D405 Video Streaming Subsystem

# Description: Capturing aligned depth and color streams 
# from the hardware sensor, converting raw frames to matrices, 
# and rendering a horizontal side by side visualization panel

# Importing Intel RealSense SDK
# to interface with hardware devices
import pyrealsense2 as rs

# Importing numerical python library
# to handle frame matrix conversions
import numpy as np

# Importing open computer vision
# library for rendering image frames
import cv2


# Executing the main function
# to run the video streaming
def main():

    # Initializing the hardware pipeline
    # instance to manage stream topologies
    pipeline = rs.pipeline()

    # Initializing stream configuration properties
    config = rs.config()

    # Wrapping pipeline profile parameters
    # to prepare for peripheral validation
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)

    # Resolving stream setup criteria
    # matching the active hardware configuration
    pipeline_profile = config.resolve(pipeline_wrapper)

    # Extracting the target camera device
    # instance from the resolved profile
    device = pipeline_profile.get_device()

    # Querying the connected peripheral device metadata name
    print(f"Connected to device: {device.get_info(rs.camera_info.name)}")

    # Enforcing native high resolution configurations
    # for the incoming 16 bit depth data stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Enforcing identical framing dimensions
    # for the companion blue green red color stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Activating the data streams
    # matching the enforced structural rules
    pipeline.start(config)

    print("Streaming started. Press 'q' in the OpenCV window to exit.")

    # Monitoring streaming frames processing
    # safely inside an error handling block
    try:

        # Extracting peripheral frame data packets
        # continuously within an infinite processing loop
        while True:

            # Halting execution flow until
            # a synchronized frame pair arrives
            frames = pipeline.wait_for_frames()

            # Isolating the specialized depth
            # component frame from the bundle
            depth_frame = frames.get_depth_frame()

            # Isolating the standard color
            # texture component from the bundle
            color_frame = frames.get_color_frame()
            
            # Dropping processing steps if
            # either target stream asset is missing
            if not depth_frame or not color_frame:
                continue

            # Transforming raw depth array
            # chunks into readable multidimensional arrays
            depth_image = np.asanyarray(depth_frame.get_data())

            # Transforming color texture buffers
            # into standard image matrix representations
            color_image = np.asanyarray(color_frame.get_data())

            # Scaling high depth variations down
            # into visible byte ranges for rendering
            depth_scaled = cv2.convertScaleAbs(depth_image, alpha=0.03)

            # Mapping the scaled raw matrix
            # into a visual pseudo color heatmap
            depth_colormap = cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)

            # Concatenating color and depth matrices
            # side by side into an aggregate display element
            images = np.hstack((color_image, depth_colormap))

            # Pushing the aggregate display element
            # directly onto an interactive user interface panel
            cv2.imshow('RealSense D405 - RGB & Depth Feed', images)
            
            # Sampling keyboard interrupts every millisecond
            # to capture immediate user termination requests
            if cv2.waitKey(1) & 0xFF == ord('q'):

                # Terminating the processing loops
                # to trigger hardware resource teardown
                break

    # Releasing physical assets cleanly
    # regardless of execution termination status
    finally:

        # Deactivating camera data streams
        # and freeing internal sensor pipelines
        pipeline.stop()

        # Closing interface panel display blocks
        # to release windows desktop resources
        cv2.destroyAllWindows()


# Direct script execution
if __name__ == "__main__":

    # Invoking the main function
    main() 
