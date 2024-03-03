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
rho = 6
theta = np.pi/180
threshold = 160 # 60
min_line_length = 70 # 60
max_line_gap = 3 # 5

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
            roi_corners = np.array([[(0-1600,height), (width/2,height/2-100), (width+1600,height)]], dtype=np.int32) # Three coordinates of the triangle
            white = (255, 255, 255)
            cv2.fillPoly(mask, roi_corners, white)

            # apply the mask
            masked_frame = cv2.bitwise_and(edgeRgbFrame, mask)
            
            ########################################################################
            # hough line filter
            # Define our parameters for Canny
            low_threshold = 175 # 50
            high_threshold = 225   # originally 100
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
            
            # group left and right
            left_line_x = []
            left_line_y = []
            right_line_x = []
            right_line_y = []
            
            # Iterate over the output "lines" and draw lines on the image copy
            if lines_check:
                for i in range(lines.shape[0]):
                    for x1,y1,x2,y2 in lines[i]:
                        if abs(x1-x2) <= 10:
                            np.delete(lines,i)
                            break
                        if abs(y2-y1) <= 10:
                            np.delete(lines,i)
                            break
                        slope = (y2-y1)/(x2-x1)
                        # ~ if slope <= 0: # <-- If the slope is negative, left group.
                            # ~ left_line_x.extend([x1, x2])
                            # ~ left_line_y.extend([y1, y2])
                        # ~ else: # <-- Otherwise, right group.
                            # ~ right_line_x.extend([x1, x2])
                            # ~ right_line_y.extend([y1, y2])
                            
            # ~ min_y = height * (3 / 5) # <-- Just below the horizon
            # ~ max_y = height # <-- The bottom of the image
            # ~ poly_left = np.poly1d(np.polyfit(left_line_y,left_line_x,deg=1))
            # ~ left_x_start = int(poly_left(max_y))
            # ~ left_x_end = int(poly_left(min_y))
            # ~ poly_right = np.poly1d(np.polyfit(right_line_y,right_line_x,deg=1))
            # ~ right_x_start = int(poly_right(max_y))
            # ~ right_x_end = int(poly_right(min_y))
            # ~ if left_x_start < 0:
                # ~ left_x_start = 0
            # ~ if right_x_start > width:
                # ~ right_x_start = width
            # ~ cv2.line(frame,(left_x_start,max_y),(left_x_end,min_y),(255,0,0),5)
            # ~ cv2.line(frame,(right_x_start,max_y),(right_x_end,min_y),(255,0,0),5)
                        
                        cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),5)
                        cv2.line(masked_frame,(x1,y1),(x2,y2),(255,0,0),5)
            
            ########################################################################
            
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 800, 600)
            cv2.namedWindow(name+"_masked", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name+"_masked", 800, 600)
            cv2.imshow(name, frame)
            cv2.imshow(name+"_masked", masked_frame)
        if cv2.waitKey(1) == ord("q"):
            break
