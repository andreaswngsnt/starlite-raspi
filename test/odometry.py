import math
import time
from threading import (Event, Thread)
from gpiozero import Motor
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Setup
baud_rate = 400000
update_interval = 1 / 100
i2c = busio.I2C(board.SCL, board.SDA, frequency = baud_rate)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
motorL = Motor(forward = 17, backward = 27)
motorR = Motor(forward = 23, backward = 24)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in degrees (counterclockwise)
    pitch is rotation around y in degrees (counterclockwise)
    yaw is rotation around z in degrees (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x * (180 / math.pi), pitch_y * (180 / math.pi), yaw_z * (180 / math.pi) # in degrees

# Global variables
accel_x = 0
accel_y = 0
accel_z = 0
speed = 0
distance = 0
roll = 0
pitch = 0
yaw = 0
initial_yaw = 0
throttle_L = 0.85
throttle_R = 0.85

# Thread functions
def display_info():
    while True:
        print("Acceleration: (%0.6f, %0.6f, %0.6f) m/s^2" % (accel_x, accel_y, accel_z))
        print("Speed: %0.6f m/s" % (speed))
        print("Distance: %0.6f m" % (distance))
        print("Rotation: (%0.6f, %0.6f, %0.6f) degrees" % (roll, pitch, yaw))
        time.sleep(0.5)
        
        if measuring_event.is_set():
            break

def correct_movement():
    global throttle_L, throttle_R

    while True:
        # Compensate movement (3 degrees)
        # If the robot veers left: speed left motor, slow right motor
        yaw_sensitivity = 3
        corrective_throttle = 0.03
        if (yaw - initial_yaw > yaw_sensitivity) or ((yaw < 0 and initial_yaw > 0) and (360 + yaw - initial_yaw > yaw_sensitivity)):
            if throttle_L + corrective_throttle < 0.99:
                throttle_L += corrective_throttle
                throttle_R -= corrective_throttle
            motorL.forward(throttle_L)
            motorR.forward(throttle_R)

        # If the robot veers right: slow left motor, speed right motor
        elif (initial_yaw - yaw > yaw_sensitivity) or ((initial_yaw < 0 and yaw > 0) and (360 + initial_yaw - yaw > yaw_sensitivity)):
            if throttle_R + corrective_throttle < 0.99:
                throttle_L -= corrective_throttle
                throttle_R += corrective_throttle
            motorL.forward(throttle_L)
            motorR.forward(throttle_R)

        time.sleep(0.5)

        if measuring_event.is_set():
            break

while True:
    # Reset readings
    accel_x = 0
    accel_y = 0
    accel_z = 0
    speed = 0
    distance = 0

    target_distance = int(input("Enter distance to travel: "))

    if target_distance <= 0:
        continue

    # Begin display thread
    measuring_event = Event()
    display_info_thread = Thread(target = display_info)
    display_info_thread.start()

    # Get initial yaw
    # initial_yaw = euler_from_quaternion(*bno.quaternion)[2]

    # Start moving
    motorL.forward(throttle_L)
    motorR.forward(throttle_R)

    # Begin movement correction thread
    #correct_movement_thread = Thread(target = correct_movement)
    #correct_movement_thread.start()

    # Keep calculating distance until target distance reached
    while distance < target_distance:
        accel_x, accel_y, accel_z = bno.acceleration
        speed += (accel_y * update_interval)
        distance += (speed * update_interval)
        # roll, pitch, yaw = euler_from_quaternion(*bno.quaternion)

        time.sleep(update_interval)

    # Stop movement correction thread
    measuring_event.set()
    #correct_movement_thread.join()

    # Stop moving
    motorL.stop()
    motorR.stop()

    # Stop display thread
    display_info_thread.join()
