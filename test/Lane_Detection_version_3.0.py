import cv2
import numpy as np
from gpiozero import Motor
from time import sleep

# Initialize motors (adjust GPIO pins as per your configuration)
motor_left = Motor(forward = 27, backward = 17)
motor_right = Motor(forward = 23, backward = 24)

# Function to control motors based on lane position
def control_motors(lane_position, frame_center):
    threshold = 50  # Adjust as needed
    if lane_position is not None:
        if lane_position < frame_center - threshold:
            # Turn left
            motor_left.backward()
            motor_right.forward()
        elif lane_position > frame_center + threshold:
            # Turn right
            motor_left.forward()
            motor_right.backward()
        else:
            # Move forward
            motor_left.forward()
            motor_right.forward()
    else:
        # Stop or implement fallback behavior
        motor_left.stop()
        motor_right.stop()

# Main function
def main():
    cap = cv2.VideoCapture(0)  # Initialize the camera (adjust index if needed)
    
    while True:
        ret, frame = cap.read()  # Read frame from camera
        
        # Convert to grayscale and apply edge detection, etc., to detect lanes
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply image processing techniques to detect lanes (e.g., Canny edge detection, Hough lines)
        # This is a simplified example; you'll need to fine-tune parameters and use more advanced techniques
        
        # Example: Canny edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Hough line detection
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)
        
        frame_center = frame.shape[1] // 2  # Compute frame center
        
        # Process detected lines (e.g., compute average lane position)
        lane_position = None  # Compute this based on detected lines
        
        # Control motors based on lane position
        control_motors(lane_position, frame_center)
        
        # Display the frame (optional)
        cv2.imshow('Lane Detection', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
