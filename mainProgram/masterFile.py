import pigpio
import sys
import os
import threading
import serial
from imuGet import getVectors

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
    uart.write(struct.pack("<9h", getVectors()))
    time.sleep(0.01)