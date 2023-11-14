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
update_interval = 1 / 200
i2c = busio.I2C(board.SCL, board.SDA, frequency = baud_rate)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
motorL = Motor(forward = 24, backward = 23)
motorR = Motor(forward = 27, backward = 17)

accel_x = 0
accel_y = 0
accel_z = 0
speed = 0
distance = 0

# Thread function
def display_info():
    while True:
        print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
        print("Speed: %0.6f m/s" % (speed))
        print("Distance: %0.6f m" % (distance))
        time.sleep(0.5)
        
        if event.is_set():
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
    event = Event()
    display_info_thread = Thread(target = display_info)
    display_info_thread.start()

    # Start moving
    motorL.forward()
    motorR.forward()

    # Keep calculating distance until target distance reached
    while distance < target_distance:
        accel_x, accel_y, accel_z = bno.acceleration
        # acceleration = accel_y
        speed += (accel_y * update_interval)
        distance += (speed * update_interval)
        
        time.sleep(update_interval)

    # Stop moving
    motorL.stop()
    motorR.stop()

    # Stop the thread
    event.set()
    display_info_thread.join()
