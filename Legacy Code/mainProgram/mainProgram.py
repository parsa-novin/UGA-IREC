import pigpio
import sys
import os
import threading
import board
import busio
import serial
import time
import struct
from getData import getVectors, getRRC3
from airbrakes import testFunction, waitAndReset

pi = pigpio.pi()

uartRadio = serial.Serial("/dev/ttyAMA3", 115200, timeout=1, write_timeout=0)
uartRRC3 = serial.Serial("/dev/ttyAMA4", 9600, timeout=1, write_timeout=0)

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
global airbrakesServo
airbrakesServo = 75
defaults = {
    'timestamp':   0,
    'altitude':    0,
    'velocity':    0,
    'temperature': 0,
    'event':       '???'
}
vals = defaults.copy()
pi.set_servo_pulsewidth(12, 1333)
packets = 0
with open("telemmytree.txt", "a") as log:
    while(True):
        stupid = getVectors()
        uartRadio.write(struct.pack("<9f", *stupid))
        print(stupid[2])
        log.write("packet #: " + str(packets) + "\n")
        log.write("ACCEL\nX: " + str(stupid[0]) + "\nY: " + str(stupid[1]) + "\nZ: " + str(stupid[2]) + "\n")
        log.write("GYRO\nX: " + str(stupid[3]) + "\nY: " + str(stupid[4]) + "\nZ: " + str(stupid[5]) + "\n")
        log.write("MAG\nX: " + str(stupid[6]) + "\nY:" + str(stupid[7]) + "\nZ: " + str(stupid[8]) + "\n")
        log.write("--------------")
        packets+=1
        if uartRRC3.in_waiting > 0:
            line = uartRRC3.read_until(expected=b'\r')
            text = line.rstrip(b'\r').decode('utf-8', errors='ignore')
            log.write("packet #: " + str(packets) + "\n")
            log.write(line + "\n")
            log.write("--------------")
            packets+=1
            vals.update(getRRC3(text))
            ev = vals['event'][:3].encode('ascii')
            if len(ev) < 3:
                ev = ev.ljust(3, b'?')
            uartRadio.write(struct.pack('<fiih3s', vals['timestamp'], vals['altitude'], vals['velocity'], vals['temperature'], ev))
            pi.set_servo_pulsewidth(12, testFunction(stupid[2], vals['timestamp']))
        if(testFunction(stupid[2], vals['timestamp']) > 1888 and vals['velocity'] > 90):
            threading.Thread(target=waitAndReset, args=(((vals['velocity']/500) * 5),), daemon=True).start()
        elif(vals['velocity'] > 90):
            print("servo actuate")
        else:
            airbrakesServo = 73
            pi.set_servo_pulsewidth(12, testFunction(stupid[2], vals['timestamp']))
        log.write("\ncurrent airbrakes step: " + str(airbrakesServo) + "\n")
        log.flush()
        time.sleep(0.025)