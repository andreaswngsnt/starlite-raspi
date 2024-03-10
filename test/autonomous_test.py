import threading
from time import sleep
from lane_detector_3 import LaneDetector
from control_system import ControlSystem

lane_detector = LaneDetector()
control_system = ControlSystem(debug_mode = False)

def handle_camera():
    lane_detector.start(camera_callback)

angle = None
obstacle_detected = False
def camera_callback(output_frame, output_angle, output_obstacle_detected):
    # Get calculated angle & output obstacle
    global angle
    angle = output_angle
    global obstacle_detected
    obstacle_detected = output_obstacle_detected

autonomous_mode = True
def handle_autonomous_navigation():
    global autonomous_mode

    # Run the code every 0.1 seconds
    while True:
        # Run these if autonomous mode
        if autonomous_mode:
            if obstacle_detected:
                control_system.stop()
                autonomous_mode = False
            else:
                # Start moving if the robot is not moving
                if control_system.throttle_L == 0 and control_system.throttle_R == 0:
                    control_system.move_forward()
                else:
                    # Adjust the steering angle, based on the lane detector
                    if angle is not None:
                        max_steering_angle = 10
                        input_steering_angle = angle - 90

                        # Limit the steering angle
                        if input_steering_angle > max_steering_angle:
                            control_system.adjust_reference_yaw(max_steering_angle)
                        elif input_steering_angle < ((-1) * max_steering_angle):
                            control_system.adjust_reference_yaw((-1) * max_steering_angle)
                        else:
                            control_system.adjust_reference_yaw(input_steering_angle)

        sleep(0.1)


camera_thread = threading.Thread(target=handle_camera)
camera_thread.start()
sleep(10)
autonomous_thread = threading.Thread(target=handle_autonomous_navigation)
autonomous_thread.start()
