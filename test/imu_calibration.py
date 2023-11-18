import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_ACTIVITY_CLASSIFIER
)
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C initialization
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)

print(bno.activity_classification)
bno.begin_calibration()
bno.save_calibration_data()

while True:
    time.sleep(1)

    if bno.calibration_status == 1:
        print("Calibration complete!")
        break

    else:
        print("Calibrating now... Status: %i" % bno.calibration_status)