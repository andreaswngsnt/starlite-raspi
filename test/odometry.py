# NOTE: The robot is moving backwards

import math
import time
from threading import (Event, Thread)
from gpiozero import Motor
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Setup
baud_rate = 400000
update_interval = 1 / 1000
i2c = busio.I2C(board.SCL, board.SDA, frequency = baud_rate)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
motorL = Motor(forward = 24, backward = 23)
motorR = Motor(forward = 27, backward = 17)

acceleration = 0
speed = 0
distance = 0

def display_info():
    while True:
        print("Acceleration: %0.6f  m/s^2" % (acceleration))
        print("Speed: %0.6f m/s" % (speed))
        print("Distance: %0.6f m" % (distance))
        time.sleep(0.5)
        
        if event.is_set():
            break

while True:
    # Reset readings
    acceleration = 0
    speed = 0
    distance = 0

    target_distance = int(input("Enter distance to travel: "))

    if target_distance <= 0:
        continue

    motorL.backward()
    motorR.backward()

    # Begin display thread
    event = Event()
    display_info_thread = Thread(target = display_info)
    display_info_thread.start()

    while distance < target_distance:
        accel_x, accel_y, accel_z = bno.acceleration
        acceleration = -1 * accel_y
        speed += (acceleration * update_interval)
        distance += (speed * update_interval)
        
        time.sleep(update_interval)

    # Stop the thread
    event.set()
    display_info_thread.join()

    motorL.stop()
    motorR.stop()
