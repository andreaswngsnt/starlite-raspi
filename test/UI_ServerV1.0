import socket
import threading
from gpiozero import Motor
from time import sleep
import keyboard
import pygame

HOST = '192.168.0.185'  # Listen on all available network interfaces
TCP_PORT = 12345  # Port to listen on for TCP

def handle_client(client_socket):
    while True:
        try:
            # Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            if data == "quit":
                break
            if not data:
                break  # If no data is received, close the connection

            # Manual Control Program (Start)
            # Setup
            motorL = Motor(forward = 27, backward = 17)
            motorR = Motor(forward = 23, backward = 24)

            # Controller
            pygame.init()
            pygame.joystick.init()

            num_controllers = pygame.joystick.get_count()

            if num_controllers > 0:
                controller = pygame.joystick.Joystick(0)
                controller.init()

                axes = controller.get_numaxes()

                print("Controller connected:", controller.get_name())
                print("Axes:", axes)
                print("Buttons:", controller.get_numbuttons())
                print("Hats:", controller.get_numhats())
                print()

                while True:
                    # Joystick control
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            running = False

                    # NOTE: Axis mapping is different in linux & windows

                    steer_axis = controller.get_axis(0)
                    backward_axis = controller.get_axis(4)
                    forward_axis = controller.get_axis(5)

                    print(f"Steering: {steer_axis:.3f}")
                    print(f"Forward: {forward_axis:.3f}")
                    print(f"Backward: {backward_axis:.3f}")

                    if forward_axis > 0.1:
                        throttle = (forward_axis + 1) / 2

                        if throttle > 0.1:
                            motorL.forward(throttle)
                            motorR.forward(throttle)

                        print("Forward Throttle:", throttle)
        
                    elif backward_axis > 0.1:
                        throttle = (backward_axis + 1) / 2

                        if throttle > 0.1:
                            motorL.backward(throttle)
                            motorR.backward(throttle)

                        print("Backward Throttle", throttle)

                    else:
                        motorL.stop()
                        motorR.stop()
                        print("Stop")

                    sleep(1)

            else:
                print("No controller detected.")
    
                # Intro message
                print("Keyboard Controls:")
                print("- W : Forward")
                print("- S : Backward")
                print("- A : Turn Left")
                print("- D : Turn right")
                print("- <space> : Stop")

                while True:
                    # TODO: Button press control
                    """ if keyboard.is_pressed('w'):
                        motorL.forward()
                        motorR.forward()
                        print("Forward")

                    elif keyboard.is_pressed('s'):
                        motorL.backward()
                        motorR.backward()
                        print("Backward")

                    else:
                        motorL.stop()
                        motorR.stop()
                        print("Stop") """

                    # Toggle control
                    x = data
                    print(f"{x}")

                    # Move forward
                    if x == 'w':
                        motorL.forward()
                        motorR.forward()
                        print("Forward")

                    # Move backward
                    elif x == 's':
                        motorL.backward()
                        motorR.backward()
                        print("Backward")

                    # Rotate left
                    elif x == 'q':
                        motorL.backward(1)
                        motorR.forward(1)
                        print("Left")

                    # Rotate right
                    elif x == 'e':
                        motorL.forward(1)
                        motorR.backward(1)
                        print("Left")

                    # Move while steering left
                    elif x == 'a':
                        motorL.forward(0.5)
                        motorR.forward(1)
                        print("Left")

                    # Move while steering right
                    elif x == 'd':
                        motorL.forward(1)
                        motorR.forward(0.5)
                        print("Right")

                    # Brake
                    elif x == ' ':
                        motorL.stop()
                        motorR.stop()
                        print("Stop")

                    sleep(1)
    

            motorL.stop()
            motorR.stop()
            pygame.quit()
            
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

if __name__ == "__main__":
    start_server()

