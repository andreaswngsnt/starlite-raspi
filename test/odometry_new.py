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

    def stop(self):
        self.motor_L.stop()
        self.motor_R.stop()

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
        self.measuring_event = None

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

    def display_info(self):
        while True:
            print("Acceleration: (%0.6f, %0.6f, %0.6f) m/s^2" % (self.accel_x - self.accel_x_offset, self.accel_y - self.accel_y_offset, self.accel_z))
            print("Speed: %0.6f m/s" % (self.speed))
            print("Distance: %0.6f m" % (self.distance))
            print("Error: %0.6f" % (self.yaw - self.reference_yaw))
            print("L: %0.6f R: %0.6f" % (self.throttle_L, self.throttle_R))
            #print("Rotation: (%0.6f, %0.6f, %0.6f) degrees" % (roll, pitch, yaw))
            time.sleep(0.5)
            
            if self.measuring_event.is_set():
                break

    def reset_measurements(self):
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.accel_x_offset = 0
        self.accel_y_offset = 0
        self.speed = 0
        self.distance = 0

    def correct_movement(self):
        while True:
            """
            Error:
            - Positive error means that the robot is veering left
            - Negative error means that the robot is veering right
            """
            error = (360 + self.yaw - self.reference_yaw) if (self.reference_yaw >= 0 and self.yaw < 0) else (self.yaw - self.reference_yaw)

            # Compensate the movement by varying the right throttle
            corrective_throttle = self.controller.get_output(error, self.update_interval)
            if (self.throttle_L + corrective_throttle > 0.01) and (self.throttle_L + corrective_throttle < 0.99):
                self.throttle_L += corrective_throttle
                self.plant.motor_L.forward(self.throttle_L)

            if (self.throttle_R - corrective_throttle > 0.01) and (self.throttle_R - corrective_throttle < 0.99):
                self.throttle_R -= corrective_throttle
                self.plant.motor_R.forward(self.throttle_R)

            time.sleep(self.update_interval)

            if self.measuring_event.is_set():
                break

    def move(self, reference_distance):
        self.reset_measurements()

        # Begin display thread
        self.measuring_event = Event()
        display_info_thread = Thread(target = self.display_info)
        display_info_thread.start()

        while self.distance < reference_distance:
            self.accel_x, self.accel_y, self.accel_z = self.sensor.bno.acceleration
            self.speed += ((self.accel_y - self.accel_y_offset)  * self.update_interval)
            self.distance += (self.speed * self.update_interval)
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)

            time.sleep(self.update_interval)

        self.plant.stop()

        # Stop movement correction thread
        self.measuring_event.set()
        self.correct_movement_thread.join()

        # Stop display thread
        display_info_thread.join()

control_system = ControlSystem()

while True:
    target_distance = int(input("Enter distance to travel: "))

    control_system.move(target_distance)