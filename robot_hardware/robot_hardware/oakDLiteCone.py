#!/usr/bin/env python3

#import cv2
import depthai as dai
import math
import numpy as np
import sys
from threading import Thread
import time
import ctypes

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import DepthCameraSectorRanges


num_ranges = 10  # number of range measurements to output from a stripe across the image

'''
Detect nearby objects with a laser-scan type stereo detection of a band of zones.
Output an array of average depths within an ROI
'''
class OakDLiteCone(Node):
    def __init__(self, display_video):
        self.display_video = display_video
        self.stop_thread = False

        if (not self.display_video):
            super().__init__("oakDLiteCone")  # if running ROS
            # Publisher for depth ranges
            self.ranges_msg = DepthCameraSectorRanges()
            self.depth_ranges_pub = self.create_publisher(DepthCameraSectorRanges, 'depth_ranges', 10)
            if (not self.display_video):
                self.depth_pub_timer = self.create_timer(0.2, self.depth_timer_cb)

    def depth_timer_cb(self):
        self.depth_ranges_pub.publish(self.ranges_msg)

    def run_camera(self):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        manipLeft = pipeline.create(dai.node.ImageManip)
        monoRight = pipeline.create(dai.node.MonoCamera)
        manipRight = pipeline.create(dai.node.ImageManip)
        stereo = pipeline.create(dai.node.StereoDepth)
        spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutSpatialData = pipeline.create(dai.node.XLinkOut)
        xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

        xoutDepth.setStreamName("depth")
        xoutSpatialData.setStreamName("spatialData")
        xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoLeft.setFps(4.0)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        monoRight.setFps(4.0)

        # Crop range for manip nodes that take each mono camera's output
        topLeft = dai.Point2f(0.0, 0.35)
        bottomRight = dai.Point2f(1.0, 0.65)
        manipLeft.initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y)
        manipLeft.setMaxOutputFrameSize(monoRight.getResolutionHeight()*monoRight.getResolutionWidth()*3)
        manipRight.initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y)
        manipRight.setMaxOutputFrameSize(monoRight.getResolutionHeight()*monoRight.getResolutionWidth()*3)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        spatialLocationCalculator.inputConfig.setWaitForMessage(False)

        # Create 10 ROIs
        for i in range(num_ranges):
            config = dai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 200
            config.depthThresholds.upperThreshold = 10000
            # define num_ranges regions of interest
            config.roi = dai.Rect(dai.Point2f(i*(1.0/num_ranges), 0.45),
                                  dai.Point2f((i+1)*(1.0/num_ranges), 0.55))
            spatialLocationCalculator.initialConfig.addROI(config)

        # Linking
        monoLeft.out.link(manipLeft.inputImage)
        monoRight.out.link(manipRight.inputImage)

        manipLeft.out.link(stereo.left)
        manipRight.out.link(stereo.right)

        spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
        stereo.depth.link(spatialLocationCalculator.inputDepth)

        spatialLocationCalculator.out.link(xoutSpatialData.input)
        xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            # device.setIrLaserDotProjectorBrightness(1000)

            # Output queue will be used to get the depth frames from the outputs defined above
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

            while True:
                if (self.display_video):
                    inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

                    depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

                    depth_downscaled = depthFrame[::4]
                    if np.all(depth_downscaled == 0):
                        min_depth = 0  # Set a default minimum depth value when all elements are zero
                    else:
                        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
                    max_depth = np.percentile(depth_downscaled, 99)
                    global depthFrameColor
                    depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
                    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                '''
                Sectors are the Regions of Interest (ROIs), to which distance is measured. We work our way across
                them left to right across image (clockwise around z axis), which is positive to negative bearing from robot.
                FOV of OAK-D-Lite is 73 degrees horizontally (36.5 degrees from sector -1.0 to 0 and from 0 to 1.0)
                '''
                min_depth_sector = 1.0
                current_sector = 1.0
                min_depth = 100.0
                ranges = []

                spatialData = spatialCalcQueue.get().getSpatialLocations()
                for depthData in spatialData:
                    roi = depthData.config.roi
                    # FIXME replace hard-coded width, height magic numbers with something
                    #roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                    roi = roi.denormalize(width=640, height=120)

                    xmin = int(roi.topLeft().x)
                    ymin = int(roi.topLeft().y)
                    xmax = int(roi.bottomRight().x)
                    ymax = int(roi.bottomRight().y)

                    # coords.x, y, z are distances in mm to the object in the ROI
                    coords = depthData.spatialCoordinates
                    # distance to object in ROI in meters. Reports zero if no object found
                    distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2) / 1000.0
                    ranges.append(distance)

                    # save the smallest range seen so far (ignoring zero), and which sector it's in
                    if (distance > 0.01 and distance <  min_depth):
                        min_depth = distance
                        min_depth_sector = current_sector
                    current_sector -= 2.0 / num_ranges

                    # print('{:.1f} '.format(distance), end=' ')

                    if (self.display_video):
                        color = (0,200,40)
                        fontType = cv2.FONT_HERSHEY_TRIPLEX
                        cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=2)
                        cv2.putText(depthFrameColor, "{:.1f}m".format(distance), (xmin + 10, ymin + 20), fontType, 0.6, color)

                min_depth_rad = min_depth_sector * (36.5/180) * math.pi
                # print('min depth: {:.1f} in sector {:.1f}, {:.2f} rad'.format(min_depth, min_depth_sector, min_depth_rad))

                # send output data from the measurement
                if (self.display_video):
                    # Show the frame
                    cv2.imshow("depth", depthFrameColor)
                    if cv2.waitKey(1) == ord('q'):
                        break
                else:
                    self.ranges_msg.sector_ranges = ranges
                    self.ranges_msg.min_range = min_depth
                    self.ranges_msg.min_range_rad = min_depth_rad

                if (self.stop_thread):
                    print('stop_thread is set')
                    break

'''
With no arguments, main starts a ROS node. Arguments:
-i:  main() sets display_video True and starts the video display and does not start ROS.
-i must be run from within the depthai virtual environment (source ~/src/depthai-python/env/bin/activate)
ROS and video display are mutually exclusive owing to the use of cv2 by the video display.
cv2 is compiled with numpy v1 whereas ROS is compiled with numpy v2.
'''
def main():
    display_video = False
    arg_count = len(sys.argv)
    if (arg_count > 1):
        for arg in sys.argv[1:]:
            if (arg == '-i'):
                display_video = True
    
    if (not display_video):
        rclpy.init()    # start ROS if we are not displaying video
    
    nh = OakDLiteCone(display_video)
    camera_thread = Thread(target=nh.run_camera)
    camera_thread.start()

    try:
        if (display_video):
            while True:
                time.sleep(1.0)
            camera_thread.join()
        else:
            rclpy.spin(nh)
    except KeyboardInterrupt:
        print('joining camera thread')
        nh.stop_thread = True
        camera_thread.join()

if __name__ == '__main__':
    import cv2
    main()
