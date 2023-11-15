import time
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C initialization
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.begin_calibration()

while True:
    time.sleep(1)

    if bno.calibration_status == 1:
        print("Calibration complete!")
        break

    else:
        print("Calibrating now... Status: %i" % bno.calibration_status)