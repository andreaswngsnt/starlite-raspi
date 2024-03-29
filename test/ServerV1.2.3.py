import socket
import threading
#from gpiozero import Motor
from time import sleep
import keyboard
#import pygame

#Camera Feed:
import cv2
import pickle
import struct

HOST = '127.0.0.1'  # Listen on all available network interfaces
TCP_PORT = 12345  # Port to listen on for TCP


# Manual Control Program (Start)
# Setup
#motorL = Motor(forward = 27, backward = 17)
#motorR = Motor(forward = 23, backward = 24)

def handle_client(client_socket):
    while True:
        try:
            # Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            if data == "quit":
                break
            if not data:
                break  # If no data is received, close the connection
            
            # Toggle control
            
            # Move forward
            if data == 'w':
                #motorL.forward()
                #motorR.forward()
                print("Forward")
    
            # Move backward
            elif data == 's':
                #motorL.backward()
                #motorR.backward()
                print("Backward")
    
            # Rotate left
            elif data == 'q':
                #motorL.backward(1)
                #motorR.forward(1)
                print("Rotate left")
    
            # Rotate right
            elif data == 'e':
                #motorL.forward(1)
                #motorR.backward(1)
                print("Rotate right")
    
            # Move while steering left
            elif data == 'a':
                #motorL.forward(0.5)
                #motorR.forward(1)
                print("Left")
    
            # Move while steering right
            elif data == 'd':
                #motorL.forward(1)
                #motorR.forward(0.5)
                print("Right")
    
            # Brake
            elif data == ' ':
                #motorL.stop()
                #motorR.stop()
                print("Stop")
            
            # Manual Control Program (End)

        except ConnectionResetError:
            break

    client_socket.close()

def start_server():
    server_socket_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_TCP.bind((HOST, TCP_PORT))
    server_socket_TCP.listen(5)

    print("Server is listening on...")
    print("IP Address: %s" % HOST)
    print("Port: %d" % TCP_PORT)

    while True:
        client_socket, _ = server_socket_TCP.accept()
        print("\nStatus Alert: A user is logging into the server.")

        # Start a new thread to handle the client
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

        # Initialize OpenCV video capture
        cameraIndex = 0 #Type of camera to use (i.e., "0" means laptop webcam)
        cap = cv2.VideoCapture(0)

        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Serialize frame
            data = pickle.dumps(frame)
            
            # Get the size of the data and send it
            message_size = struct.pack("L", len(data))
            client_socket.sendall(message_size + data)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    start_server()
