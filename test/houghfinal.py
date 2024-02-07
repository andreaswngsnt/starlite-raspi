import depthai as dai
import numpy as np
import cv2
from datetime import timedelta
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image

# Create pipeline
pipeline = dai.Pipeline()

# Define the Hough transform parameters
rho = 1
theta = np.pi/180
threshold = 60
min_line_length = 50
max_line_gap = 5

# Set color pipeline
# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
edgeDetectorRgb = pipeline.create(dai.node.EdgeDetector)
sync = pipeline.create(dai.node.Sync)

# Link nodes
xoutEdgeRgb = pipeline.create(dai.node.XLinkOut)
xinEdgeCfg = pipeline.create(dai.node.XLinkIn)

# Group Output
xoutGrp = pipeline.create(dai.node.XLinkOut)

xoutGrp.setStreamName("xout")

# Set camera and name
camRgb.setCamera("color")
edgeRgbStr = "edge rgb"
edgeCfgStr = "edge cfg"

sync.setSyncThreshold(timedelta(milliseconds=5))

# Linking
camRgb.video.link(sync.inputs["video"])

# Streaming
xoutEdgeRgb.setStreamName(edgeRgbStr)
xinEdgeCfg.setStreamName(edgeCfgStr)

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P) # originally 1080

edgeDetectorRgb.setMaxOutputFrameSize(camRgb.getVideoWidth() * camRgb.getVideoHeight())

# Linking
camRgb.video.link(edgeDetectorRgb.inputImage)
edgeDetectorRgb.outputImage.link(xoutEdgeRgb.input)
xinEdgeCfg.out.link(edgeDetectorRgb.inputConfig)
sync.out.link(xoutGrp.input)

with dai.Device(pipeline) as device:
    queue = device.getOutputQueue("xout", 10, False)
    while True:
        msgGrp = queue.get()
        for name, msg in msgGrp:
            frame = msg.getCvFrame()
            
            #####################################
            # for edge
            # Output queues
            edgeRgbQueue = device.getOutputQueue(edgeRgbStr, 8, False)
            edgeCfgQueue = device.getInputQueue(edgeCfgStr)
            edgeRgb = edgeRgbQueue.get()
            edgeRgbFrame = edgeRgb.getFrame()
            
            # sobel mask
            cfg = dai.EdgeDetectorConfig()
            sobelHorizontalKernel = [[1, 0, -1], [2, 0, -2], [1, 0, -1]]
            sobelVerticalKernel = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]]
            cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel)
            edgeCfgQueue.send(cfg)
            #####################################
            
            # frame width and height
            width  = edgeRgb.getWidth()
            height = edgeRgb.getHeight()
            
            # mask
            mask = np.zeros(edgeRgbFrame.shape, dtype=np.uint8)
            roi_corners = np.array([[(0-160,height), (width/2,height/2-100), (width+160,height)]], dtype=np.int32) # Three coordinates of the triangle
            white = (255, 255, 255)
            cv2.fillPoly(mask, roi_corners, white)

            # apply the mask
            masked_frame = cv2.bitwise_and(edgeRgbFrame, mask)
            
            ########################################################################
            # hough line filter
            # Define our parameters for Canny
            low_threshold = 100 # 50
            high_threshold = 200   # originally 100
            edges = cv2.Canny(masked_frame, low_threshold, high_threshold)
             
            # The below for loop runs till r and theta values
            # are in the range of the 2d array
            # Run Hough on the edge-detected image
            # ~ new_frame = np.copy(frame) #creating an image copy to draw lines on
            
            lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                    min_line_length, max_line_gap)
            # ~ print(type(lines))
            # check if lines is empty
            lines_check = np.any(lines)
            # Iterate over the output "lines" and draw lines on the image copy
            if lines_check:
                for line in lines:
                    for x1,y1,x2,y2 in line:
                        cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),5)
                        cv2.line(masked_frame,(x1,y1),(x2,y2),(255,0,0),5)
            ########################################################################
            
            cv2.imshow(name, frame)
            cv2.imshow(name+"_masked", masked_frame)
        if cv2.waitKey(1) == ord("q"):
            break
