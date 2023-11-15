from gpiozero import Motor
from time import sleep
import keyboard
import pygame

# Setup
motorL = Motor(forward = 17, backward = 27)
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
        x = input("Enter command:")
        
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