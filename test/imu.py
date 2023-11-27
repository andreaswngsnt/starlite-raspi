import time
import math
from threading import (Event, Thread)
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Sensor: Read & Compute off the sensor readings
class Sensor:
    def __init__(self, update_interval):
        self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA, frequency = 400000))
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
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
            print("Gyroscope: (%0.6f, %0.6f, %0.6f) m/s^2" % self.bno.gyro)
            print("Magnetometer: (%0.6f, %0.6f, %0.6f) m/s^2" % self.bno.magnetic)
            print("Quaternion: (%0.6f, %0.6f, %0.6f, %0.6f) m/s^2" % self.bno.quaternion)
            print("Rotation: (%0.6f, %0.6f, %0.6f) degrees" % self.get_angles_degrees())

            if callback is not None:
                callback(self.get_acceleration(), self.get_speed(), self.get_distance(), self.get_angles_degrees())

            print("")

            time.sleep(0.5)
            
            if self.active_event.is_set():
                break

    def compute_speed_distance(self):
        while True:
            self.speed += (self.get_acceleration()[1]  * self.update_interval)
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


sensor = Sensor(1/100)
sensor.activate()