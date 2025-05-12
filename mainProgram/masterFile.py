import pigpio
import threading
import board
import busio
import serial
import struct
import time
from imuGet import getVectors
from airbrakes import testFunction, waitAndReset
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)

i2c  = busio.I2C(board.D1, board.D0)
imu  = BNO08X_I2C(i2c, address=0x4A)
servo_enabled = True


pi = pigpio.pi()
pi.set_servo_pulsewidth(12, 500 + (2000 * 75) // 180)
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
airbrakesServo = 75
while(True):
    uartRadio.write(struct.pack("<9h", getVectors()))
    time.sleep(0.01)
    pi.set_servo_pulsewidth(12, testFunction(az, time))
    if(testFunction(az, time) > 1888 and velocity > 90):
        threading.Thread(target=waitAndReset, args=(((velocity/500) * 5),), daemon=True).start()
    elif(velocity > 90):
        print("servo actuate")
    else:
        airbrakesServo = 73
        pi.set_servo_pulsewidth(12, testFunction(az, time))