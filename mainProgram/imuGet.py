import time
import board
import busio
import serial
import struct
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)

i2c  = busio.I2C(board.D1, board.D0)
imu  = BNO08X_I2C(i2c, address=0x4A)
def getVectors():
    while True:
        try:
            a = imu.acceleration
            g = imu.gyro
            m = imu.magnetic
        except RuntimeError:
            continue          # skip if the driver was busy parsing
        
        line = (f"{a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f};"
                f"{g[0]:+.2f},{g[1]:+.2f},{g[2]:+.2f};"
                f"{m[0]:+.1f},{m[1]:+.1f},{m[2]:+.1f}\r\n")
        
        vals = (
            int(a[0]*1000), int(a[1]*1000), int(a[2]*1000),
            int(g[0]*1000), int(g[1]*1000), int(g[2]*1000),
            int(m[0]*1000), int(m[1]*1000), int(m[3]*1000),)
        try:
            return(vals)
            print(line)
        except serial.SerialTimeoutException:
            pass              # UART TX buffer full ? drop this sample

        time.sleep(0.01)
