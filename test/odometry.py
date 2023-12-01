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

# PID Controller: Takes in error and computes the required throttles
class PIDController:
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.__integrated_error = 0
        self.__prev_error = 0
        self.output = 0

    def reset(self):
        self.__integrated_error = 0
        self.__prev_error = 0

    def get_output(self, error, update_interval):
        self.__integrated_error += (error * update_interval)
        derivative_error = error - self.__prev_error
        self.__prev_error = error
        return self.k_p * error + self.k_i * self.__integrated_error + self.k_d * (derivative_error / update_interval)
    


# Plant: Takes in throttle values & moves the motors
class Plant:
    def __init__(self):
        self.motor_L = Motor(forward = 27, backward = 17)
        self.motor_R = Motor(forward = 23, backward = 24)

    def move(self, throttle_L, throttle_R):
        if throttle_L >= 0:
            if throttle_L > 1:
                self.motor_L.forward(1)
            else:
                self.motor_L.forward(throttle_L)
        else:
            if throttle_L < -1:
                self.motor_L.backward(1)
            else:
                self.motor_L.backward(-1 * throttle_L)
        
        if throttle_R >= 0:
            if throttle_R > 1:
                self.motor_R.forward(1)
            else:
                self.motor_R.forward(throttle_R)
        else:
            if throttle_R < -1:
                self.motor_R.backward(1)
            else:
                self.motor_R.backward(-1 * throttle_R)

    def stop(self):
        self.motor_L.stop()
        self.motor_R.stop()

    def __del__(self):
        self.motor_L.stop()
        self.motor_R.stop()



# Sensor: Read & Compute off the sensor readings
class Sensor:
    def __init__(self, update_interval):
        self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA, frequency = 400000))
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.update_interval = update_interval
        self.accel_x_offset = 0
        self.accel_y_offset = 0
        self.accel_z_offset = 0
        self.speed = 0
        self.distance = 0
        self.active_event = None
        self.display_info_thread = None
        self.compute_speed_distance_thread = None

    def activate(self, display_callback = None):
        self.active_event = Event()

        # Calibrate accelerometer
        #self.accel_x_offset = self.bno.acceleration[0]
        #self.accel_y_offset = self.bno.acceleration[1]

        # Create & start compute thread
        self.compute_speed_distance_thread = Thread(target = self.compute_speed_distance)
        self.compute_speed_distance_thread.start()
        
        # Create & start display thread
        if display_callback is not None:
            self.display_info_thread = Thread(target = self.display_info, args = [display_callback])
        else:
            self.display_info_thread = Thread(target = self.display_info)
        self.display_info_thread.start()

    def deactivate(self):
        self.active_event.set()
        self.display_info_thread.join()
        self.compute_speed_distance_thread.join()

    def reset(self):
        self.accel_x_offset = 0
        self.accel_y_offset = 0
        self.accel_z_offset = 0
        self.speed = 0
        self.distance = 0

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

    def display_info(self, callback = None):
        while True:
            print("Acceleration: (%0.6f, %0.6f, %0.6f) m/s^2" % self.get_acceleration())
            print("Speed: %0.6f m/s" % self.speed)
            print("Distance: %0.6f m" % self.distance)
            print("Rotation: (%0.6f, %0.6f, %0.6f) degrees" % self.get_angles_degrees())

            if callback is not None:
                callback(self.get_acceleration(), self.get_speed(), self.get_distance(), self.get_angles_degrees())

            print("")

            time.sleep(0.5)
            
            if self.active_event.is_set():
                break

    def compute_speed_distance(self):
        while True:
            self.speed += (self.get_acceleration()[0]  * self.update_interval)
            self.distance += (self.speed * self.update_interval)
            time.sleep(self.update_interval)

            if self.active_event.is_set():
                break
    
    def get_acceleration(self):
        return self.bno.acceleration[0] - self.accel_x_offset, self.bno.acceleration[1] - self.accel_y_offset, self.bno.acceleration[2] - self.accel_z_offset
    def get_speed(self):
        return self.speed
    def get_distance(self):
        return self.distance
    def get_quaternion(self):
        return self.bno.quaternion
    def get_angles_degrees(self):
        return self.euler_from_quaternion(*self.bno.quaternion)



class ControlSystem:
    def __init__(self):
        """
        PID values that work
        0.0001, 0, 0.0001
        """
        self.controller = PIDController(0.0001, 0, 0.0001)
        self.plant = Plant()
        self.sensor = Sensor(1 / 100)
        self.update_interval = 1 / 100
        
        self.reference_yaw = 0
        self.throttle_L = 0
        self.throttle_R = 0

        # "Global" coordinates
        self.x = 0
        self.y = 0
        self.d_prev = 0
        self.zero_yaw = 0
        self.theta = 0
        self.theta_prev = 0

        self.active_event = None
        self.localization_thread = None

    def display_extra_info(self, acceleration, speed, distance, degrees):
        print("Yaw Error: %0.6f" % self.compute_yaw_error())
        print("L: %0.6f R: %0.6f" % (self.throttle_L, self.throttle_R))
        print("Rotation: (%0.6f, %0.6f, %0.6f) degrees" % degrees)
        print("Global Location: (%0.6f, %0.6f)" % (self.x, self.y))
        print("Global Yaw: %0.6f" % self.theta)

    def reset_throttle(self):
        self.throttle_L = 0
        self.throttle_R = 0

    def compute_yaw_error(self):
        """
        Error:
        - Positive error means that the rover needs to turn CCW
        - Negative error means that the rover needs to turn CW
        """
        if self.reference_yaw - self.sensor.get_angles_degrees()[2] <= -180:
            return self.reference_yaw - self.sensor.get_angles_degrees()[2] + 360
        elif self.reference_yaw - self.sensor.get_angles_degrees()[2] > 180:
            return self.reference_yaw - self.sensor.get_angles_degrees()[2] - 360
        else:
            return self.reference_yaw - self.sensor.get_angles_degrees()[2]

    def correct_yaw(self):
        while True:
            # Correct the yaw by changing the throttles
            corrective_throttle = self.controller.get_output(self.compute_yaw_error(), self.update_interval)
            self.throttle_L -= corrective_throttle
            self.throttle_R += corrective_throttle

            time.sleep(self.update_interval)

            if self.active_event.is_set():
                break

    # TODO: Fix this shit
    def localization(self):
        while True:
            # Update global thetas
            self.theta_prev = self.theta
            if self.sensor.get_angles_degrees()[2] - self.zero_yaw <= -180:
                self.theta = self.sensor.get_angles_degrees()[2] - self.zero_yaw + 360
            elif self.sensor.get_angles_degrees()[2] - self.zero_yaw  > 180:
                self.theta = self.sensor.get_angles_degrees()[2] - self.zero_yaw - 360
            else:
                self.theta = self.sensor.get_angles_degrees()[2] - self.zero_yaw

            # Update x & y coordinates
            d_delta = self.sensor.get_distance() - self.d_prev
            d_theta = self.theta - self.theta_prev
            self.x += d_delta * math.cos((self.theta  * (math.pi / 180) + d_theta * (math.pi / 180) / 2))
            self.y += d_delta * math.sin((self.theta  * (math.pi / 180) + d_theta * (math.pi / 180) / 2))
            self.d_prev = self.sensor.get_distance()

            time.sleep(self.update_interval)

    def move(self, reference_distance):
        self.throttle_L = 0.8
        self.throttle_R = 0.8

        # Get initial readings & offset
        self.reference_yaw = self.sensor.get_angles_degrees()[2]
        
        # Activate sensor
        self.sensor.activate(self.display_extra_info)

        # Begin yaw correction thread
        self.active_event = Event()
        correct_yaw_thread = Thread(target = self.correct_yaw)
        correct_yaw_thread.start()

        # Keep calculating distance until target distance reached
        while self.sensor.get_distance() < reference_distance:
            # Move
            self.plant.move(self.throttle_L, self.throttle_R)

            time.sleep(self.update_interval)

        self.plant.stop()

        # Stop yaw correction thread
        self.active_event.set()
        correct_yaw_thread.join()

        # Deactivate sensor & reset readings
        self.sensor.deactivate()
        self.sensor.reset()
        self.d_prev = 0

        self.reset_throttle()

    def activate(self):
        # Wait for the IMU to initialize
        time.sleep(0.5)
        self.zero_yaw = self.sensor.get_angles_degrees()[2]
        self.localization_thread = Thread(target = self.localization)
        self.localization_thread.start()

    def rotate(self, reference_angle):
        self.reset_throttle()

        # Activate the sensor
        self.sensor.activate(self.display_extra_info)

        # Get initial readings & set reference yaw
        initial_yaw = self.sensor.get_angles_degrees()[2]
        if initial_yaw + reference_angle >= 180:
            self.reference_yaw = initial_yaw + reference_angle - 360
        elif initial_yaw + reference_angle <= -180:
            self.reference_yaw = initial_yaw + reference_angle + 360
        else:
            self.reference_yaw = initial_yaw + reference_angle

        print("Initial yaw: %0.6f" % initial_yaw)
        print("Reference yaw: %0.6f" % self.reference_yaw)

        # Begin yaw correction thread
        self.active_event = Event()
        correct_yaw_thread = Thread(target = self.correct_yaw)
        correct_yaw_thread.start()
        
        while True:
            # If target yaw reached & is in steady state, stop rotating
            if self.compute_yaw_error() > -1 and self.compute_yaw_error() < 1:
                time.sleep(self.update_interval * 10)
                if self.compute_yaw_error() > -1 and self.compute_yaw_error() < 1:
                    break

            # Rotate
            self.plant.move(self.throttle_L, self.throttle_R)

            time.sleep(self.update_interval)

        self.plant.stop()
        
        # Stop yaw correction thread
        self.active_event.set()
        correct_yaw_thread.join()

        # Deactivate sensor & reset readings
        self.sensor.deactivate()
        self.sensor.reset()
        self.d_prev = 0

        print("Reference yaw: %0.6f" % self.reference_yaw)
        print("Final yaw: %0.6f" % self.sensor.get_angles_degrees()[2])
        print("Final error: %0.6f" % self.compute_yaw_error())

        self.reset_throttle()

    def __del__(self):
        self.localization_thread.join()



control_system = ControlSystem()
control_system.activate()

while True:
    target_distance = float(input("Enter distance to travel: "))
    if target_distance < 0:
        continue

    target_angle = float(input("Enter angle to face: "))
    if target_angle > 180 or target_angle <= -180:
        continue

    control_system.rotate(target_angle)
    print("")
    print("")
    control_system.move(target_distance)
    print("")
    print("")