from gpiozero import Motor

# Setup
motorL = Motor(forward = 24, backward = 23)
motorR = Motor(forward = 27, backward = 17)

# Moves robot
# Throttle : 0.1 to 1 (forward) or -0.1 to -1 (backward)
# Angle: -45 to 45 degree (turn left to right)
def move(throttle, angle):
    if throttle > 0.1:
        if angle == 0:
            motorL.forward(throttle)
            motorR.forward(throttle)
        elif angle < 0 and angle >= -45:
            motorL.forward(throttle / 2)
            motorR.forward(throttle)
        elif angle > 0 and angle <= 45:
            motorL.forward(throttle)
            motorR.forward(throttle / 2)
    elif throttle < 0.1:
        if angle == 0:
            motorL.backward(-1 * throttle)
            motorR.backward(-1 * throttle)
        elif angle < 0 and angle >= -45:
            motorL.backward(-1 * throttle / 2)
            motorR.backward(-1 * throttle)
        elif angle > 0 and angle <= 45:
            motorL.backward(-1 * throttle)
            motorR.backward(-1 * throttle / 2)

def stop():
    motorL.stop()
    motorR.stop()