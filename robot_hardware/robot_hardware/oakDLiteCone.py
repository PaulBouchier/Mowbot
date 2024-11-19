#!/usr/bin/env python3

import cv2
import depthai as dai
import math
import numpy as np
import sys

def make_display_frame():
    inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

    depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

    depth_downscaled = depthFrame[::4]
    if np.all(depth_downscaled == 0):
        min_depth = 0  # Set a default minimum depth value when all elements are zero
    else:
        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
    max_depth = np.percentile(depth_downscaled, 99)
    depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

    return depthFrameColor

display_video = False
arg_count = len(sys.argv)
if (arg_count > 1):
    for arg in sys.argv[1:]:
        if (arg == '-i'):
            display_video = True

'''
Detect nearby objects with a laser-scan type stereo detection of a band of zones.
Output an array of average depths within an ROI
'''
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
for i in range(10):
    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 200
    config.depthThresholds.upperThreshold = 10000
    config.roi = dai.Rect(dai.Point2f(i*0.1, 0.45), dai.Point2f((i+1)*0.1, 0.55))
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
    color = (0,200,40)
    fontType = cv2.FONT_HERSHEY_TRIPLEX

    depthFrameColor = make_display_frame()

    while True:
        if (display_video):
            depthFrameColor = make_display_frame()

        min_depth_sector = 0
        current_sector = 0
        min_depth = 10000.0

        spatialData = spatialCalcQueue.get().getSpatialLocations()
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)

            # save the smallest range seen so far, and which sector it's in
            if (distance <  min_depth):
                min_depth = distance
                min_depth_sector = current_sector
            current_sector += 1

            print('{:.1f} '.format(distance/1000), end=' ')

            if (display_video):
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=2)
                cv2.putText(depthFrameColor, "{:.1f}m".format(distance/1000), (xmin + 10, ymin + 20), fontType, 0.6, color)
        # Show the frame
        print('min depth: {:.1f} in sector {}'.format(min_depth/1000, min_depth_sector))
       
        if (display_video):
            cv2.imshow("depth", depthFrameColor)

        if cv2.waitKey(1) == ord('q'):
            break