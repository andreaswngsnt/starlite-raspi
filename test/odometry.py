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

    def move(self, throttle_L, throttle_R):
        if throttle_L > 1:
            self.motor_L.forward(1)
        elif throttle_L < 0:
            self.motor_L.backward(-1 * throttle_L)
        else:
            self.motor_L.forward(throttle_L)

        if throttle_R > 1:
            self.motor_R.forward(1)
        elif throttle_R < 0:
            self.motor_R.backward(-1 * throttle_R)
        else:
            self.motor_R.forward(throttle_R)

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
        self.controller = PIDController(0.0001, 0, 0.0001)
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

    def euler_from_quaternion(self, x, y, z, w):
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
            print("Acceleration: (%0.6f, %0.6f, %0.6f) m/s^2" % (self.accel_x, self.accel_y, self.accel_z))
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
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def reset_throttle(self):
        self.throttle_L = 0
        self.throttle_R = 0

    def correct_yaw(self):
        while True:
            """
            Error:
            - Positive error means that the robot is too left
            - Negative error means that the robot is too right
            """
            error = (360 + self.yaw - self.reference_yaw) if (self.reference_yaw >= 0 and self.yaw < 0) else (self.yaw - self.reference_yaw)

            # Correct the yaw by changing the throttles
            corrective_throttle = self.controller.get_output(error, self.update_interval)
            self.throttle_L += corrective_throttle
            self.throttle_R -= corrective_throttle

            time.sleep(self.update_interval)

            if self.measuring_event.is_set():
                break

    def move(self, reference_distance):
        self.reset_measurements()
        self.throttle_L = 0.8
        self.throttle_R = 0.8

        # Get initial readings & offset
        self.accel_x_offset = self.sensor.bno.acceleration[0]
        self.accel_y_offset = self.sensor.bno.acceleration[1]
        self.reference_yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)[2]
        
        # Begin display thread
        self.measuring_event = Event()
        display_info_thread = Thread(target = self.display_info)
        display_info_thread.start()

        # Begin yaw correction thread
        correct_yaw_thread = Thread(target = self.correct_yaw)
        correct_yaw_thread.start()

        # Keep calculating distance until target distance reached
        while self.distance < reference_distance:
            # Move
            self.plant.move(self.throttle_L, self.throttle_R)

            # Read off sensors
            self.accel_x, self.accel_y, self.accel_z = self.sensor.bno.acceleration
            self.speed += ((self.accel_y)  * self.update_interval)
            self.distance += (self.speed * self.update_interval)
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)

            time.sleep(self.update_interval)

        self.plant.stop()

        # Stop yaw correction thread
        self.measuring_event.set()
        correct_yaw_thread.join()

        # Stop display thread
        display_info_thread.join()

    def rotate(self, reference_angle):
        self.reset_measurements()
        self.reset_throttle()

        # Get initial readings & set reference yaw
        self.yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)[2]
        if self.yaw + reference_angle >= 180:
            self.reference_yaw = self.yaw + reference_angle - 360
        elif self.yaw + reference_angle <= -180:
            self.reference_yaw = self.yaw + reference_angle + 360
        else:
            self.reference_yaw = self.yaw + reference_angle

        # Begin display thread
        self.measuring_event = Event()
        display_info_thread = Thread(target = self.display_info)
        display_info_thread.start()

        # Begin yaw correction thread
        correct_yaw_thread = Thread(target = self.correct_yaw)
        correct_yaw_thread.start()

        # Keep reading off sensor until target yaw reached
        error = (360 + self.yaw - self.reference_yaw) if (self.reference_yaw >= 0 and self.yaw < 0) else (self.yaw - self.reference_yaw)
        while error > 1:
            # Rotate
            self.plant.move(self.throttle_L, self.throttle_R)

            # Read off sensors
            self.accel_x, self.accel_y, self.accel_z = self.sensor.bno.acceleration
            self.speed += ((self.accel_y)  * self.update_interval)
            self.distance += (self.speed * self.update_interval)
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(*self.sensor.bno.quaternion)

            time.sleep(self.update_interval)

        self.plant.stop()
        
        # Stop yaw correction thread
        self.measuring_event.set()
        correct_yaw_thread.join()

        # Stop display thread
        display_info_thread.join()



control_system = ControlSystem()

while True:
    target_distance = int(input("Enter distance to travel: "))
    if target_distance <= 0:
        continue

    target_angle = float(input("Enter angle to face: "))
    if target_angle > 180 or target_angle <= -180:
        continue

    control_system.rotate(target_angle)
    control_system.move(target_distance)