################################################################################
#Libraries:

#1. Client-Server Architecture:
import socket

#2. Simultaneous Processing:
import threading

#3. Manual & Autonomous Control:
from time import sleep
from lane_detector_3 import LaneDetector

#4. Camera Feed:
import cv2
import struct

#5. Control System
from control_system import ControlSystem

################################################################################
#Source Code Documentation:

#Starlite Delivery Robot
#Last Updated: 03/07/2024 at 7:00 PM

################################################################################
#Global Variables:

#Server Information:
HOST = '192.168.0.185'  #Server IP (Use '127.0.0.1' for local testing)
TCP_PORT = 12345  #Server Port (TCP)
UDP_PORT = 12346 #Server Port (UDP)

################################################################################
#Module Initializations:

#Motor Control:
control_system = ControlSystem(False)

################################################################################
#Main Control Program (Start):

#Autonomous Control:
autonomous_mode = False

#Function to receive manual control inputs:
def handle_client(client_socket):
    global autonomous_mode
    
    while True:
        try:
            #Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            
            if data == "quit":
                break #If user wants to terminate, close the connection.
            if not data:
                break  #If no data is received, close the connection.
            
            #Motor Control Functionalities:

            #Manual Control Program (Start):
            
            #Move forward
            if (data == 'w') & (autonomous_mode == False):
                control_system.move_forward()
                print("Forward")
    
            #Move backward
            elif (data == 's') & (autonomous_mode == False):
                control_system.move_backward()
                print("Backward")
    
            #Move while steering left
            elif (data == 'a') & (autonomous_mode == False):
                control_system.rotate_left()
                print("Left")
    
            #Move while steering right
            elif (data == 'd') & (autonomous_mode == False):
                control_system.rotate_right()
                print("Right")
    
            #Brake
            elif (data == ' ') & (autonomous_mode == False):
                control_system.stop()
                print("Stop")

            #Manual Control Program (End)

            #Autonomous Control Program (Start):
            elif data == 'on':
                autonomous_mode = True
                print("Autonomous mode activated.")

            #Autonomous Control Program (End)

            #Control Toggle:
            elif data == 'off':
                autonomous_mode = False
                print("Autonomous mode deactivated.")
            
            #Main Control Program (End)

        except ConnectionResetError:
            autonomous_mode = False
            break

    #Closing the connection:
    client_socket.close()

################################################################################
#Camera Transmission:

# Data from the camera
angle = None
obstacle_detected = False

#Function to run the camera and take its annotated frame
def handle_camera(server_socket_UDP):
    lane_detector = LaneDetector()
    lane_detector.start(camera_callback, server_socket_UDP)

# Callback function, called at every frame outputted by the lane detector
def camera_callback(output_frame, output_angle, output_obstacle_detected, server_socket_UDP):
    # Get calculated angle & output obstacle
    global angle
    angle = output_angle
    global obstacle_detected
    obstacle_detected = output_obstacle_detected

    # Resizing frame:
    frame75 = rescale_frame(output_frame, percent = 10)

    # Send the frame using UDP
    _, img_encoded = cv2.imencode('.jpg', frame75)
    frame_bytes = img_encoded.tobytes()

    # Send the frame size first
    frame_size = len(frame_bytes)

    server_socket_UDP.sendto(struct.pack("!I", len(frame_bytes)), (HOST, UDP_PORT))

    # Send the frame to the client
    server_socket_UDP.sendto(frame_bytes, (HOST, UDP_PORT))

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

################################################################################
#Autonomous Navigation (Start):
        
# Function for autonomous navigation
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

#Autonomous Navigation (End)

################################################################################
#Server Management:

#Function to start the server:
def start_server():
    #Initializing the server socket (TCP):
    server_socket_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_TCP.bind((HOST, TCP_PORT))

    #Initializing the server socket (UDP):
    server_socket_UDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket_UDP.bind((HOST, UDP_PORT))

    #Listening for connection requests (maximum number of simultaneous connections = 5):
    server_socket_TCP.listen(5)

    #Displaying server information:
    print("Server is listening on...")
    print("IP Address: %s" % HOST)
    print("TCP Port: %d" % TCP_PORT)
    print("UDP Port: %d" % UDP_PORT)

    #Performing background tasks:
    while True:
        #Accepting connection requests:
        client_socket, _ = server_socket_TCP.accept()

        #Displaying a message whenever users log in:
        print("\nStatus Alert: A user is logging into the server.")

        #Starting a new thread to receive manual control inputs:
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

        #Starting a new thread to run camera:
        camera_thread = threading.Thread(target=handle_camera, args=(client_socket,))
        camera_thread.start()
        
        sleep(10)
        
        #Start a new thread for autonomous navigation:
        autonomous_thread = threading.Thread(target=handle_autonomous_navigation)
        autonomous_thread.start()

#Program Execution:
if __name__ == "__main__":
    start_server()

#End of File
