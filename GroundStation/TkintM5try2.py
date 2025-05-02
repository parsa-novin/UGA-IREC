import tkinter as tk
from tkinter import ttk

import struct 

import random
import time

import serial.tools.list_ports

def getCOMS():

    ports = serial.tools.list_ports.comports()

    portList = []

    for onePort in ports:
        portList.append(str(onePort))
        print(str(onePort))

    val = input("Select port: COM")
    for x in range(0, len(portList)):
        if portList[x].startswith("COM" + str(val)):
            portVar = "COM" + str(val)
            print(portList[x])
    return portVar

serialInst = serial.Serial()
serialInst.baudrate = 115200
serialInst.port = getCOMS()

root = tk.Tk()
label = tk.Label(root, text="Initial Text")
label.pack()



i = 0
serialInst.open()
def read_serial():
    global i
    if serialInst.in_waiting:
        i += 1
        packet = serialInst.readline()
        label.config(text = struct.unpack('fff', packet))
    
    root.after(1, read_serial)

read_serial()
root.mainloop()
    
