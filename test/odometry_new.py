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

class PIDController:
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.__integrated_error = 0
        self.__prev_error = 0

    def get_output(self, error, update_interval):
        self.__integrated_error += (error * update_interval)
        derivative_error = error - self.__prev_error
        self.__prev_error = error
        return self.k_p * error + self.k_i * self.__integrated_error + self.k_d * (derivative_error / update_interval)
    
    def reset(self):
        self.__integrated_error = 0
        self.__prev_error = 0

class Plant:
    def __init__(self):
        self.motor_L = Motor(forward = 27, backward = 17)
        self.motor_R = Motor(forward = 23, backward = 24)

    def __del__(self):
        self.motor_L.stop()
        self.motor_R.stop()

class Sensor:
    def __init__(self):
        self.bno = BNO08X_I2C(
            busio.I2C(board.SCL, board.SDA, frequency = 400000)
            )
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

class ControlSystem:
    def __init__(self):
        self.controller = PIDController(0.00005, 0, 1)
        self.plant = Plant()
        self.sensor = Sensor()
        self.update_interval = 1 / 100
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.accel_x_offset = 0
        self.accel_y_offset = 0
        self.speed = 0
        self.distance = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.reference_yaw = 0
        self.throttle_L = 0.75
        self.throttle_R = 0.75

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

    def move(self, reference_distance):
        while self.distance < reference_distance:
            self.accel_x, self.accel_y, self.accel_z = self.sensor.bno.acceleration
            self.speed += ((self.accel_y - self.accel_y_offset)  * self.update_interval)
            self.distance += (self.speed * self.update_interval)
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)

            time.sleep(self.update_interval)

control_system = ControlSystem()

while True:
    target_distance = int(input("Enter distance to travel: "))

    control_system.move(target_distance)

