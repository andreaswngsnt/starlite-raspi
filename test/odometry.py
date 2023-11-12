from gpiozero import Motor
from time import sleep
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Setup
baud_rate = 400000
update_interval = 1 / 200000
i2c = busio.I2C(board.SCL, board.SDA, frequency = baud_rate)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
motorL = Motor(forward = 24, backward = 23)
motorR = Motor(forward = 27, backward = 17)

while True:
    target_distance = input("Enter distance to travel:")

    if target_distance <= 0:
        continue

    speed = 0
    distance = 0
    time_elapsed = 0

    motorL.forward()
    motorR.forward()

    while distance < target_distance:
        sleep(update_interval)
        accel_x, accel_y, accel_z = bno.acceleration
        speed += (accel_y * update_interval)
        distance += (speed * update_interval)
        time_elapsed += 1

        if time_elapsed % 100000 == 0:
            print("Acceleration: %0.6f m/s^2" % (accel_y))
            print("Speed: %0.6f m/s" % (speed))
            print("Distance: %0.6f m" % (distance))

    motorL.stop()
    motorR.stop()
