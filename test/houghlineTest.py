import depthai as dai
import numpy as np
import cv2
from datetime import timedelta
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
from PIL import Image

# Create pipeline
pipeline = dai.Pipeline()

# Define the Hough transform parameters
rho = 6
theta = np.pi/60
threshold = 160 # 60
min_line_length = 40 # 60
max_line_gap = 5 # 5

# Set color pipeline
# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
# ~ camRgb.setVideoSize(1920, 1080)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)

# function for angle calculation
def angle(x1, y1, x2, y2):
    radian = math.atan2((y2 - y1) ** 2, (x2 - x1) ** 2)
    angle = radian * (180/math.pi)
    # ~ if angle < 0:
        # ~ angle += 360
    angle = round(angle, 2)
    return angle

with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    while True:
        videoIn = video.get()
        RGBframe = videoIn.getCvFrame()
        
        # frame width and height
        width  = RGBframe.shape[1]
        height = RGBframe.shape[0]
        # testing
        # ~ print("---------")
        # ~ print(width)
        # ~ print(height)
        # ~ print(RGBframe.shape)
        # ~ print("---------")
        
        # Convert to grayscale here.
        grayFrame = cv2.cvtColor(RGBframe, cv2.COLOR_RGB2GRAY)
        
        ########################################################################
        # hough line filter
        # Define our parameters for Canny
        low_threshold = 50 # 175
        high_threshold = 200   # originally 225
        cannyFrame = cv2.Canny(grayFrame, low_threshold, high_threshold)
        
        # ROI
        roi = np.zeros(cannyFrame.shape, dtype=np.uint8)
        roi_corners = np.array([[(0-1600,height), (width/2,height/2), (width+1600,height)]], dtype=np.int32) # Three coordinates of the triangle
        white = (255, 255, 255)
        cv2.fillPoly(roi, roi_corners, white)

        # apply the ROI
        roiFrame = cv2.bitwise_and(cannyFrame, roi)
       
        # The below for loop runs till r and theta values
        # are in the range of the 2d array
        # Run Hough on the edge-detected image
      
        lines = cv2.HoughLinesP(roiFrame, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)
        # check if lines is empty
        lines_check = np.any(lines)
        
        # group lines
        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []
      
        # Iterate over the output "lines" and draw lines on the image copy
        if lines_check:
            for i in range(lines.shape[0]):
                for x1,y1,x2,y2 in lines[i]:
                    if abs(y2-y1) <= 10:
                        continue
                    if abs(x2-x1) <= 10:
                        continue
                    if x2 <= width * 5/7 and x2 >= width * 3/7:
                        continue
                    slope = (y2-y1)/(x2-x1)
                    if math.fabs(slope) < 0.5:
                        continue
                    cv2.line(RGBframe,(x1,y1),(x2,y2),(0,0,255),5)
                    # ~ cv2.line(roiFrame,(x1,y1),(x2,y2),(255,0,0),5)
                    if slope <= 0: # <-- If the slope is negative, left group.
                        left_line_x.extend([x1, x2])
                        left_line_y.extend([y1, y2])
                    else: # <-- Otherwise, right group.
                        right_line_x.extend([x1, x2])
                        right_line_y.extend([y1, y2])
        
        # poly fit the line
        min_y = roiFrame.shape[0] * (3 / 5) # <-- Just below the horizon
        max_y = roiFrame.shape[0] # <-- The bottom of the image
        
        # check if left has line
        checkLeft = False
        if left_line_x and left_line_y:
            checkLeft = True
            poly_left = np.poly1d(np.polyfit(
                left_line_y,
                left_line_x,
                deg=1
            ))
            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
            # draw line
            cv2.line(RGBframe,(left_x_start,max_y),(left_x_end,int(min_y)),(255,0,0),5)
            cv2.line(roiFrame,(left_x_start,max_y),(left_x_end,int(min_y)),(255,0,0),5)
        
        # check if right has line
        checkRight = False
        if right_line_x and right_line_y:
            checkRight = True
            poly_right = np.poly1d(np.polyfit(
                right_line_y,
                right_line_x,
                deg=1
            ))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))
            # draw line
            cv2.line(RGBframe,(right_x_start,max_y),(right_x_end,int(min_y)),(255,0,0),5)
            cv2.line(roiFrame,(right_x_start,max_y),(right_x_end,int(min_y)),(255,0,0),5)
        
        # get direction
        directionExist = False
        if checkLeft and checkRight:
            directionExist = True
            lane_x_start = int((left_x_start + right_x_start) / 2)
            lane_y_start = max_y
            lane_x_end = int((left_x_end + right_x_end) / 2)
            lane_y_end = min_y
            
            # testing
            # ~ print("---------")
            # ~ print(lane_x_start)
            # ~ print(lane_y_start)
            # ~ print(lane_x_end)
            # ~ print(lane_y_end)
            # ~ print("---------")
            
            cv2.line(RGBframe,(lane_x_start,max_y),(lane_x_end,int(min_y)),(0,255,0),5)
            # calculate the angle
            radian = math.atan2((0 - (height-lane_y_end)), (lane_x_start - lane_x_end))
            angle = radian * (180/math.pi)
            if angle < 0:
                angle = angle + 180
            angle = round(angle, 2)
            # ~ theta = angle(lane_x_end,(height-lane_y_end),lane_x_start,0)
            # ~ print("---------")
            print(angle)
            # ~ print("---------")
            # ~ cv2.line(roiFrame,(lane_x_start,max_y),(lane_x_end,int(min_y)),(0,255,0),5)
            
        # ~ # calculate the angle
        # ~ if directionExist:
            # ~ # fixing the coordinate
            # ~ n_x_start = lane_x_end # n stands for normalized
            # ~ n_y_start = height - lane_y_end
            # ~ n_x_end = lane_x_start
            # ~ n_y_end = height - lane_y_start
            
            # ~ theta = angle(n_x_start,n_y_start,n_x_end,n_y_end)
            
            # ~ # testing
            # ~ print("---------")
            # ~ print(n_x_start)
            # ~ print(n_y_start)
            # ~ print(n_x_end)
            # ~ print(n_y_end)
            # ~ print(str(theta)+" degree")
            # ~ print("---------")

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.namedWindow("video", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("video", 800, 600)
        cv2.imshow("video", RGBframe)
        
        # test for masked frame
        cv2.namedWindow("roi", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("roi", 800, 600)
        cv2.imshow("roi", roiFrame)


        if cv2.waitKey(1) == ord('q'):
            break
