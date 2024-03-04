################################################################################
#Libraries:

#1. Client-Server Architecture:
import socket

#2. Simultaneous Processing:
import threading

#3. Manual Control:
from time import sleep
import keyboard

#4. Camera Feed:
import cv2
import pickle
import struct
import numpy as np
import time

from lane_detector import LaneDetector

#5. Control System
from control_system import ControlSystem

################################################################################
#Source Code:

#Server Information:
HOST = '192.168.0.185'  #Server IP (Use '127.0.0.1' for local testing)
TCP_PORT = 12345  #Server Port
UDP_PORT = 12346


control_system = ControlSystem(False)
autonomous_mode = False

#Manual Control Program (Start):

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
                autonomous_mode = False
                control_system.move_forward()
                print("Forward")
    
            #Move backward
            elif data == 's':
                autonomous_mode = False
                control_system.move_backward()
                print("Backward")
    
            #Move while steering left
            elif data == 'a':
                autonomous_mode = False
                control_system.rotate_left()
                print("Left")
    
            #Move while steering right
            elif data == 'd':
                autonomous_mode = False
                control_system.rotate_right()
                print("Right")
    
            #Brake
            elif data == ' ':
                autonomous_mode = False
                control_system.stop()
                print("Stop")
            
            #Manual Control Program (End)

        except ConnectionResetError:
            autonomous_mode = False
            break

    #Closing the connection:
    client_socket.close()


# Data from the camera
annotated_frame = None
angle = None

#Function to run the camera and take its annotated frame
def handle_camera():
    
    lane_detector = LaneDetector()
    lane_detector.start(assign_camera_outputs)

def assign_camera_outputs(output_frame, output_angle):
    annotated_frame = output_frame
    angle = output_angle

#Function to transmit camera feed from the Raspberry Pi to user interface:
def handle_send_UDP_frame(server_socket_UDP):
    while True:
        if annotated_frame is not None:
            _, img_encoded = cv2.imencode('.jpg', annotated_frame)
            frame_bytes = img_encoded.tobytes()

            # Send the frame size first
            frame_size = len(frame_bytes)
            #client_socket.sendall(frame_size.to_bytes(4, byteorder='big')) //TCP

            server_socket_UDP.sendto(struct.pack("!I", len(frame_bytes)), (HOST, UDP_PORT))

            # Send the frame to the client
            #client_socket.sendall(frame_bytes) //TCP

            server_socket_UDP.sendto(frame_bytes, (HOST, UDP_PORT))
            
            print(angle)
        
        time.sleep(0.1)    
        
        
def handle_autonomous_navigation():
    while True:
        if autonomous_mode:
            control_system.move_forward()
            if angle is not None:
                control_system.adjust_reference_yaw(angle - 90)
                
        control_system.stop()
        time.sleep(0.25)
    

#Function to start the server:
def start_server():
    #Initializing the server socket (TCP):
    server_socket_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_TCP.bind((HOST, TCP_PORT))

    server_socket_UDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket_UDP.bind((HOST, UDP_PORT))

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

        #Starting a new thread to run camera:
        camera_thread = threading.Thread(target=handle_camera)
        camera_thread.start()
        
        #Start a new thread for autonomous navigation:
        autonomous_thread = threading.Thread(target=handle_autonomous_navigation)
        autonomous_thread.start()
        
        #TODO:Starting a new thread to transmit camera feed:
        send_UDP_frame_thread = threading.Thread(target=handle_send_UDP_frame, args=(client_socket,))
        send_UDP_frame_thread.start()

#Program Execution:
if __name__ == "__main__":
    start_server()
