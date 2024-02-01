import depthai as dai
import numpy as np
import cv2
from datetime import timedelta
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image

pipeline = dai.Pipeline()

# Define the Hough transform parameters
rho = 1
theta = np.pi/180
threshold = 60
min_line_length = 50
max_line_gap = 5

# Set color pipeline
# ~ monoLeft = pipeline.create(dai.node.MonoCamera)
# ~ monoRight = pipeline.create(dai.node.MonoCamera)
color = pipeline.create(dai.node.ColorCamera)
# ~ stereo = pipeline.create(dai.node.StereoDepth)
sync = pipeline.create(dai.node.Sync)

# Group Output
xoutGrp = pipeline.create(dai.node.XLinkOut)

xoutGrp.setStreamName("xout")

# ~ monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# ~ monoLeft.setCamera("left")
# ~ monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# ~ monoRight.setCamera("right")

# ~ stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# Set camera and name
color.setCamera("color")

sync.setSyncThreshold(timedelta(milliseconds=5))

# ~ monoLeft.out.link(stereo.left)
# ~ monoRight.out.link(stereo.right)

# Linking
# ~ stereo.disparity.link(sync.inputs["disparity"])
color.video.link(sync.inputs["video"])

sync.out.link(xoutGrp.input)

# ~ disparityMultiplier = 255.0 / stereo.initialConfig.getMaxDisparity()
with dai.Device(pipeline) as device:
    queue = device.getOutputQueue("xout", 10, False)
    while True:
        msgGrp = queue.get()
        for name, msg in msgGrp:
            frame = msg.getCvFrame()
            
            # frame width and height
            width  = color.getIspWidth()
            height = color.getIspHeight()
            
            # mask
            mask = np.zeros(frame.shape, dtype=np.uint8)
            roi_corners = np.array([[(0,height), (width/2,height/2), (width,height)]], dtype=np.int32)#Three co ordinates of the triangle
            white = (255, 255, 255)
            cv2.fillPoly(mask, roi_corners)

            # apply the mask
            masked_frame = cv2.bitwise_and(frame, mask)

            
            # ~ # Convert the img to grayscale
            # ~ gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
             
            # ~ # Apply edge detection method on the image
            # ~ edges = cv2.Canny(gray, 50, 150, apertureSize=3)
             
            # ~ # This returns an array of r and theta values
            # ~ lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
            
            # Convert image to grayscale
            gray = cv2.cvtColor(masked_frame, cv2.COLOR_RGB2GRAY)
            # Define our parameters for Canny
            low_threshold = 50
            high_threshold = 150    # originally 100
            edges = cv2.Canny(gray, low_threshold, high_threshold)
             
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
            
            cv2.imshow(name, frame)
            cv2.imshow(name+"_masked", masked_frame)
        if cv2.waitKey(1) == ord("q"):
            break
