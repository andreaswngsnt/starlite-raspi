################################################################################
#Libraries:

#1. Client-Server Architecture:
import socket

#2. Simultaneous Processing:
import threading

#3. Manual Control:
#from control_system import ControlSystem
#from gpiozero import Motor
from time import sleep
import keyboard

#4. Camera Feed:
import cv2
import pickle
import struct
#import depthai as dai
import numpy as np
#from datetime import timedelta

################################################################################
#Source Code:

#Camera Setup:
pipeline = dai.Pipeline()

# Set color pipeline
color = pipeline.create(dai.node.ColorCamera)
sync = pipeline.create(dai.node.Sync)

# Group Output
xoutGrp = pipeline.create(dai.node.XLinkOut)

xoutGrp.setStreamName("xout")

# Set camera and name
color.setCamera("color")

sync.setSyncThreshold(timedelta(milliseconds=50))

# Linking
# ~ stereo.disparity.link(sync.inputs["disparity"])
color.video.link(sync.inputs["video"])

sync.out.link(xoutGrp.input)

#Server Information:
HOST = '127.0.0.1'  #Server IP (Use '127.0.0.1' for local testing and '192.168.0.185' to connect to robot)
TCP_PORT = 12345  #Server Port


#Manual Control Program (Start):

#Function to receive manual control inputs:
def handle_client(client_socket):
    #Control System Initialization:
    #control_system = ControlSystem()
    
    while True:
        try:
            #Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            
            if data == "quit":
                break #If user wants to terminate, close the connection.
            if not data:
                break  #If no data is received, close the connection.
            
            #Motor Control Functionalities:
            
            #Move forward:
            if data == 'w':
                #control_system.move_forward()
                print("Forward")
    
            #Move backward:
            elif data == 's':
                #control_system.move_backward()
                print("Backward")
    
            #Rotate left:
            elif data == 'a':
                #control_system.rotate_left()
                print("Left")
    
            #Rotate right:
            elif data == 'd':
                #control_system.rotate_right()
                print("Right")
    
            #Stop:
            elif data == ' ':
                #control_system.stop()
                print("Stop")
            
            #Manual Control Program (End)

        #If the connection is reset, close the connection:
        except ConnectionResetError:
            break

    #Closing the connection:
    client_socket.close()

#Function to transmit camera feed from the Raspberry Pi to user interface:
def handle_camera(client_socket):
    #Depth Camera Test:
    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue("xout", 10, False)
        while True:
            msgGrp = queue.get()
            for name, msg in msgGrp:
                frame = msg.getCvFrame()
                data = pickle.dumps(frame)
                
                #Getting the size of the data and sending it:
                message_size = struct.pack("L", len(data))
                client_socket.sendall(message_size + data)

                #Display camera feed for debugging purposes:
                cv2.imshow(name, frame)

            #If "q" is pressed, quit sending camera feed:
            if cv2.waitKey(1) == ord("q"):
                break

# Function to control motors based on lane position (autonomous navigation module):
def control_motors(lane_position, frame_center):
    threshold = 50  # Adjust as needed
    if lane_position is not None:
        if lane_position < frame_center - threshold:
            #Turning left:
            #control_system.rotate_left()
            print("Turning left...")
        elif lane_position > frame_center + threshold:
            #Turning right:
            #control_system.rotate_right()
            print("Turning right...")
        else:
            #Moving forward:
            #control_system.move_forward()
            print("Moving foward...")
    else:
        #Stopping or implementing fallback behavior:
        #control_system.stop()
        print("Stopping...")
        
#Function to start the server:
def start_server():
    #Initializing the server socket (TCP):
    server_socket_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_TCP.bind((HOST, TCP_PORT))

    #Listening for connection requests (maximum number of simultaneous connections = 5):
    server_socket_TCP.listen(5)

    #Displaying server information:
    print("Server is listening on...")
    print("IP Address: %s" % HOST)
    print("Port: %d" % TCP_PORT)

    #Performing background tasks:
    while True:
        #Accepting connection requests:
        client_socket, _ = server_socket_TCP.accept()

        #Displaying a message whenever users log in:
        print("\nStatus Alert: A user is logging into the server.")

        #Starting a new thread to receive manual control inputs:
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

        #Starting a new thread to transmit camera feed:
        camera_thread = threading.Thread(target=handle_camera, args=(client_socket,))
        camera_thread.start()

        #Starting a new thread to enable lane detection (autonomous navigation module):
        lane_detection_thread = threading.Thread(target=control_motors, args=(lane_position, frame_center,))
        lane_detection_thread.start()

#Program Execution:
if __name__ == "__main__":
    start_server()
