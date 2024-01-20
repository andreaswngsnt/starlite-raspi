from control_system import ControlSystem

control_system = ControlSystem()

# Intro message
print("Keyboard Controls:")
print("- W : Forward")
print("- S : Backward")
print("- A : Turn Left")
print("- D : Turn right")
print("- <space> : Stop")

while True:
    # Toggle control
    x = input("Enter command:")
    
    # Move forward
    if x == 'w':
        control_system.move_forward()
        print("Forward")

    # Move backward
    elif x == 's':
        control_system.move_backward()
        print("Backward")

    # Rotate left
    elif x == 'a':
        control_system.rotate_left()
        print("Left")

    # Rotate right
    elif x == 'd':
        control_system.rotate_right()
        print("Right")

    # Brake
    elif x == ' ':
        control_system.stop()
        print("Stop")

    sleep(1)