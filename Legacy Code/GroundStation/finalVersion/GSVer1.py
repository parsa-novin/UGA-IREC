import csv
import random
import time

import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()


portList = []

for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

val = input("Select port: COM")
for x in range(0, len(portList)):
    if portList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print(portList[x])

serialInst.baudrate = 9600
serialInst.port = portVar

serialInst.open()

time_val = 0
alt_val = 0
velo_val = 0
temp_val = 0
event_1 = 0
event_2 = 0
fuckUps = 0





fieldNames = ["TimeStamp", "Altitude", "Velocity", "Temperature", "Event1", "Event2"]

with open('Data.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames = fieldNames)
    csv_writer.writeheader()

while True:
    with open('Data.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldNames)

        info = {"TimeStamp": time_val,
                    "Altitude": alt_val,
                    "Velocity": velo_val,
                    "Temperature": temp_val,
                    "Event1": event_1,
                    "Event2": event_2,
            }
        if serialInst.in_waiting:
            nastyData = serialInst.readline()
            dirtyData = nastyData.decode('utf-8')
            
            data = dirtyData.strip(',???\r\nDM ')

            try:
                int_list = [float(x) for x in data.split(',')]

                csv_writer.writerow(info)

                time_val = int_list[0]
                alt_val = int_list[1]
                velo_val = int_list[2]
                temp_val = int_list[3]
                event_1 = 0
                event_2 = 0

            except: 
                fuckUps += 1


        

    time.sleep(1)

