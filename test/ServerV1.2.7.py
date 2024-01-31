################################################################################
#Libraries:

#1. Client-Server Architecture:
import socket

#2. Simultaneous Processing:
import threading

#3. Manual Control:
#from gpiozero import Motor
from time import sleep
import keyboard

#4. Camera Feed:
import cv2
import pickle
import struct
import depthai as dai
import numpy as np
from datetime import timedelta

################################################################################
#Source Code:

#Camera setup:
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
HOST = '192.168.0.185'  #Server IP (Use '127.0.0.1' for local testing)
TCP_PORT = 12345  #Server Port


#Manual Control Program (Start):

#Motor Setup:
#motorL = Motor(forward = 27, backward = 17)
#motorR = Motor(forward = 23, backward = 24)

#Function to receive manual control inputs:
def handle_client(client_socket):
    while True:
        try:
            #Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            
            if data == "quit":
                break #If user wants to terminate, close the connection.
            if not data:
                break  #If no data is received, close the connection.
            
            #Motor Control Functionalities:
            
            #Move forward
            if data == 'w':
                #motorL.forward()
                #motorR.forward()
                print("Forward")
    
            #Move backward
            elif data == 's':
                #motorL.backward()
                #motorR.backward()
                print("Backward")
    
            #Rotate left
            elif data == 'q':
                #motorL.backward(1)
                #motorR.forward(1)
                print("Rotate left")
    
            #Rotate right
            elif data == 'e':
                #motorL.forward(1)
                #motorR.backward(1)
                print("Rotate right")
    
            #Move while steering left
            elif data == 'a':
                #motorL.forward(0.5)
                #motorR.forward(1)
                print("Left")
    
            #Move while steering right
            elif data == 'd':
                #motorL.forward(1)
                #motorR.forward(0.5)
                print("Right")
    
            #Brake
            elif data == ' ':
                #motorL.stop()
                #motorR.stop()
                print("Stop")
            
            #Manual Control Program (End)

        except ConnectionResetError:
            break

    #Closing the connection:
    client_socket.close()

#Function to transmit camera feed from the Raspberry Pi to user interface:
def handle_camera(client_socket):
    # ~ #Initializing OpenCV video capture:
    # ~ cameraIndex = 0 #Type of camera to use (i.e., "0" means laptop webcam)
    # ~ cap = cv2.VideoCapture(0) #Turn on camera

    # ~ #Video-frame Processing:
    # ~ while True:
        # ~ #Capture video frame-by-frame:
        # ~ ret, frame = cap.read()
        
        # ~ #Serialize frame:
        # ~ data = pickle.dumps(frame)
        
        # ~ #Getting the size of the data and sending it:
        # ~ message_size = struct.pack("L", len(data))
        # ~ client_socket.sendall(message_size + data)
        
        # ~ # Break the loop if 'q' is pressed
        # ~ if cv2.waitKey(1) & 0xFF == ord('q'):
            # ~ break
    
    #Depth-Camera Test:
    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue("xout", 10, False)
        while True:
            msgGrp = queue.get()
            for name, msg in msgGrp:
                # ~ print(type(msg))
                frame = msg.getCvFrame()
                
                #Serialize frame:
                data = pickle.dumps(frame)
        
                #Getting the size of the data and sending it:
                message_size = struct.pack("L", len(data))
                client_socket.sendall(message_size + data)

                #Displaying the server camera-feed:
                cv2.imshow(name, frame)
                
            if cv2.waitKey(1) == ord("q"):
                break

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

#Program Execution:
if __name__ == "__main__":
    start_server()
