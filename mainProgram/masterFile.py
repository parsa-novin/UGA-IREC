import pigpio
import sys
import os
import threading
import board
import busio
import serial
import struct
from imuGet import getVectors
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)

i2c  = busio.I2C(board.D1, board.D0)
imu  = BNO08X_I2C(i2c, address=0x4A)



pi = pigpio.pi()

uartRadio = serial.Serial("/dev/ttyAMA3", 115200, timeout=1, write_timeout=0)

ADC_SER_ENABLE = 19
DES_ENABLE = 13
E_STOP = 26
RADIO_RTS = 21
RADIO_CTS = 20
MANN_CLK = 6
AIRBRAKES = 12

#I2C ADDRESSES
# 0x4A = IMU
# 0x6A = BMS
# 0x77 = baro
# 0x3D = SER

pi.set_mode(AIRBRAKES, pigpio.ALT0)
pi.set_mode(MANN_CLK, pigpio.ALT0)
pi.set_mode(E_STOP, pigpio.OUTPUT)

while(True):
    uartRadio.write(struct.pack("<9h", getVectors()))
    time.sleep(0.01)