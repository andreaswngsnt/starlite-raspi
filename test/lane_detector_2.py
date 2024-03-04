import depthai as dai
import numpy as np
import cv2
from datetime import timedelta
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
from PIL import Image

class LaneDetector:
    def __init__(self):
        # Create pipeline
        self.pipeline = dai.Pipeline()

        # TODO: Test depth functionality
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setCamera("left")
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setCamera("right")
        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)

        # Define the Hough transform parameters
        self.rho = 6
        self.theta = np.pi/60
        self.threshold = 160 # 60
        self.min_line_length = 60 # 60
        self.max_line_gap = 5 # 5
        
        # Set color pipeline
        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.camRgb.setCamera("color")
        self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)
        
        self.xoutVideo.setStreamName("video")

        # Properties
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        self.xoutVideo.input.setBlocking(False)
        self.xoutVideo.input.setQueueSize(1)

        # TODO: Sync depth & color inputs
        self.sync = self.pipeline.create(dai.node.Sync)
        self.sync.setSyncThreshold(timedelta(milliseconds=50))
        self.stereo.disparity.link(self.sync.inputs["disparity"])
        self.camRgb.video.link(self.sync.inputs["video"])

        self.sync.out.link(self.xoutVideo.input)
        self.disparityMultiplier = 255.0 / self.stereo.initialConfig.getMaxDisparity()
        
        # Outputs
        self.angle = None
        self.annotated_frame = None
        
    def get_angle(self):
        return self.angle
        
    def get_annotated_frame(self):
        return self.annotated_frame
        
        
    def start(self, callback = None, *args):
        with dai.Device(self.pipeline) as device:
            queue = device.getOutputQueue(name="video", maxSize=1, blocking=False)

            while True:
                msgGrp = queue.get()

                for name, msg in msgGrp:
                    frame = msg.getCvFrame()
                    
                    if name == "video":
                        # frame width and height
                        width  = frame.shape[1]
                        height = frame.shape[0]
                        
                        # Convert to grayscale here.
                        grayFrame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                        
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
                    
                        lines = cv2.HoughLinesP(roiFrame, self.rho, self.theta, self.threshold, np.array([]),
                                                self.min_line_length, self.max_line_gap)
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
                                    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),5)
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
                            cv2.line(frame,(left_x_start,max_y),(left_x_end,int(min_y)),(255,0,0),5)
                            # ~ cv2.line(roiFrame,(left_x_start,max_y),(left_x_end,int(min_y)),(255,0,0),5)
                        
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
                            cv2.line(frame,(right_x_start,max_y),(right_x_end,int(min_y)),(255,0,0),5)
                            # ~ cv2.line(roiFrame,(right_x_start,max_y),(right_x_end,int(min_y)),(255,0,0),5)
                        
                        # get direction
                        directionExist = False
                        if checkLeft and checkRight:
                            directionExist = True
                            lane_x_start = int((left_x_start + right_x_start) / 2)
                            lane_y_start = max_y
                            lane_x_end = int((left_x_end + right_x_end) / 2)
                            lane_y_end = min_y
                            
                            cv2.line(frame,(lane_x_start,max_y),(lane_x_end,int(min_y)),(0,255,0),5)
                            self.annotated_frame = frame
                            
                            # calculate the angle
                            radian = math.atan2((0 - (height-lane_y_end)), (lane_x_start - lane_x_end))
                            angle = radian * (180/math.pi)
                            if angle < 0:
                                angle = angle + 180
                            self.angle = round(angle, 2)

                    # TODO: Depth test
                    if name == "disparity":
                        frame = (frame * self.disparityMultiplier).astype(np.uint8)
                        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

                    # Get BGR frame from NV12 encoded video frame to show with opencv
                    # Visualizing the frame on slower hosts might have overhead
                    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(name, 800, 600)
                    cv2.imshow(name, frame)

                    # Callback function
                    if callback is not None:
                        callback(self.annotated_frame, self.angle, *args)

                if cv2.waitKey(1) == ord('q'):
                    break
