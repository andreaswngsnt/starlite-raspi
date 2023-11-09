import socket
import threading
from gpiozero import Motor
from time import sleep
import keyboard
import pygame

# Server configuration
HOST = '192.168.0.185'  # Listen on all available network interfaces
PORT = 12345      # Port to listen on
USERS = {'user1': 'password1', 'user2': 'password2'}  # Set the username and password

def authenticate(username, password):
    return USERS.get(username) == password

def handle_client(client_socket):
    client_address = client_socket.getpeername()
    print("Connection established with", client_address)

    try:
        while True:
            username = client_socket.recv(1024).decode('utf-8')
            password = client_socket.recv(1024).decode('utf-8')

            if not username or not password:
                break  # Empty credentials, disconnect the client

            if authenticate(username, password):
                client_socket.send(b"Correct")
            else:
                client_socket.send(b"Wrong")

            # Setup
            motorL = Motor(forward = 24, backward = 23)
            motorR = Motor(forward = 27, backward = 17)

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
                    x = client_socket.recv(1024).decode('utf-8')
                    print(x)

                    if x == 'w':
                        motorL.forward()
                        motorR.forward()
                        print("Forward")

                    elif x == 's':
                        motorL.backward()
                        motorR.backward()
                        print("Backward")

                    elif x == 'a':
                        motorL.forward(0.5)
                        motorR.forward(1)
                        print("Left")

                    elif x == 'd':
                        motorL.forward(1)
                        motorR.forward(0.5)
                        print("Right")

                    elif x == ' ':
                        motorL.stop()
                        motorR.stop()
                        print("Stop")

                    sleep(1)
    

            motorL.stop()
            motorR.stop()
            pygame.quit()
                
    except Exception as e:
        print("Error:", e)
    finally:
        client_socket.close()
        print("Connection closed with", client_address)

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)

    print("Server is listening on...")
    print("IP Address: %s" % HOST)
    print("Port: %d" % PORT)

    while True:
        client_socket, _ = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

if __name__ == "__main__":
    start_server()

