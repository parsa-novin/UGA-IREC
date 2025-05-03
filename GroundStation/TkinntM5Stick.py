import tkinter as tk
from tkinter import ttk
import random
import struct
import time
import serial.tools.list_ports
import math

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
'''
def unit(v):
    x, y, z = v
    mag = math.sqrt(x*x + y*y + z*z)
    return (x/mag, y/mag, z/mag) if mag else (0,0,0)

def angle_between(v1, v2):
    # normalise
    ux, uy, uz = unit(v1)
    vx, vy, vz = unit(v2)

    # dot product
    dot = ux*vx + uy*vy + uz*vz

    # numerical safety
    dot = max(-1.0, min(1.0, dot))

    # radians → degrees
    return math.degrees(math.acos(dot))
'''
class DashboardApp:
    FMT_IMU      = '<9f'
    FMT_FLIGHT   = '<fiih3s'
    SZ_IMU       = struct.calcsize(FMT_IMU)
    SZ_FLIGHT    = struct.calcsize(FMT_FLIGHT)
    def __init__(self, parent, portVar):
        self.log = open("groundtelemmy.txt","a",buffering=1)
        self.ser = serial.Serial(portVar, 115200, timeout=0.5)
        self.parent = parent
        self.portVar = portVar
        self.packets = 0
        #fill the screen
        parent.state('zoomed')

        self.parent.title("UGA IREC Telemetry Dashboard")

        # Create a Frame to hold the dashboard widgets
        self.dashboard_frame = ttk.Frame(parent)
        self.dashboard_frame.grid(sticky="nsew")

        #make labels updatable
        self.time_value = tk.Label(self.dashboard_frame, text= "UNCHANGE" + " Seconds", font=("Arial Black", 12))
        self.alt_value = tk.Label(parent, font=("Arial Black", 12))

        # Create and place widgets on the dashboard using the Grid geometry manager
        self.create_widgets(portVar)
        self.update_values()

    def create_widgets(self, portVar):
        # Create and place labels
        labelAlt = tk.Label(self.dashboard_frame, text="Altitude", bg = "light grey", font=("Arial Black", 16))
        labelAlt.grid(row=0, column=0, padx = 0, pady = 0, sticky = "nsew")

        labelVelo = tk.Label(self.dashboard_frame, text="Velocity", bg = "light grey", font=("Arial Black", 16))
        labelVelo.grid(row=1, column=0, sticky = "nsew")

        labelAcc = tk.Label(self.dashboard_frame, text="Acceleration", bg = "light grey", font=("Arial Black", 16))
        labelAcc.grid(row=2, column=0, sticky = "nsew")

        labelTemp = tk.Label(self.dashboard_frame, text= portVar , bg = "light grey", font=("Arial Black", 16))
        labelTemp.grid(row=3, column=0, sticky = "nsew")

        labelTime = tk.Label(self.dashboard_frame, text="Time Since Liftoff", bg = "light grey", font=("Arial Black", 16))
        labelTime.grid(row=4, column=0, sticky = "nsew")


        labelParachute = tk.Label(self.dashboard_frame, text="Parachute Status", bg = "light grey", font=("Arial Black", 16))
        labelParachute.grid(row=0, column=4, rowspan= 1, sticky ="nsew")

        self.labelDrogue = tk.Label(self.dashboard_frame, text="Drogue Status", bg = "red", font=("Arial Black", 16))
        self.labelDrogue.grid(row=1, column=4, rowspan= 1, sticky ="nsew")

        self.labelMain = tk.Label(self.dashboard_frame, text="Main Status", bg = "red", font=("Arial Black", 16))
        self.labelMain.grid(row=2, column=4, rowspan= 1, sticky ="nsew")


        labelAngAtt = tk.Label(self.dashboard_frame, text="Angle of Attack", bg = "light grey", font=("Arial Black", 16))
        labelAngAtt.grid(row=3, column=4, padx = 0, sticky = "nsew")

        labelRadVelo = tk.Label(self.dashboard_frame, text="Radial Velocity", bg = "light grey", font=("Arial Black", 16))
        labelRadVelo.grid(row=4, column=4, sticky = "nsew")



        # Create and place statistics widgets (placeholders)
        self.alt_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Feet", font=("Arial Black", 12))
        self.alt_value.grid(row=0, column=0, sticky ="s")
        
        self.velo_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " ft/s", font=("Arial Black", 12))
        self.velo_value.grid(row=1, column=0, sticky ="s")

        self.acc_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " ft/s^2", font=("Arial Black", 12))
        self.acc_value.grid(row=2, column=0, sticky ="s")

        self.temp_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " F", font=("Arial Black", 12))
        self.temp_value.grid(row=3, column=0, sticky ="s")
        
        self.time_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Seconds", font=("Arial Black", 12))
        self.time_value.grid(row=4, column=0, sticky ="s")

        self.angAtt_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Degrees", font=("Arial Black", 12))
        self.angAtt_value.grid(row=3, column=4, sticky ="s")

        self.radVelo_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " deg/sec", font=("Arial Black", 12))
        self.radVelo_value.grid(row=4, column=4, sticky ="s")

        # Create and place graph widgets (placeholders)
        # You can use Matplotlib or other libraries for more advanced graphs
        self.footage = tk.Canvas(self.dashboard_frame, bg="white", width=400, height=400)
        self.footage.grid(row=0, column=1, columnspan= 3, rowspan =3, sticky="nsew")

        self.sponsor = tk.Canvas(self.dashboard_frame, bg="grey", width=400, height=400)
        self.sponsor.grid(row=3, column=1, columnspan= 3, sticky="nsew")

        self.miniRoc = tk.Canvas(self.dashboard_frame, bg="red", width=400, height=400)
        self.miniRoc.grid(row=4, column=1, columnspan= 3, sticky="nsew")

        # Configure grid layout weights to make the dashboard responsive
        for col in range(5):
            self.dashboard_frame.columnconfigure(col, weight=1, uniform ="equal")
        for row in range(5):
            self.dashboard_frame.rowconfigure(row, weight=1, uniform ="equal")

        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(0, weight=1)
    def update_values(self):
        pkt = self.ser.read(max(self.SZ_IMU, self.SZ_FLIGHT))

        if len(pkt) == self.SZ_IMU:                     # 9‑float IMU packet
            ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack(self.FMT_IMU, pkt)

            self.acc_value .config(text=f"{az:+.2f} m/s^2")
            g = 9.80665
            ratio = min(1, abs(az)/g)
            theta_rad = math.acos(ratio)     # radians
            theta_deg = math.degrees(theta_rad)
            #theta_deg = angle_between((ax, ay, az), (mx, my, mz))
            self.angAtt_value.config(text=f"{theta_deg:5.1f} degrees")
            self.radVelo_value.config(text=f"{gz:+.1f} °/s")
            self.log.write(f"Packet # {self.packets}\n")
            self.log.write(f"Acceleration: (X: {ax:+.1f}; Y: {ay:+.1f}; Z: {az:+.1f})\n")
            self.log.write(f"Gyroscope: (X: {gx:+.1f}; Y: {gy:+.1f}; Z: {gz:+.1f})\n")
            self.log.write(f"Magnetometer: (X: {mx:+.1f}; Y: {my:+.1f}; Z: {mz:+.1f})\n")
            self.log.write("----------------------------\n")
            self.packets += 1

        elif len(pkt) == self.SZ_FLIGHT:
            t, alt, vel, temp, dma = struct.unpack(self.FMT_FLIGHT, pkt)
            dma = dma.decode('ascii')

            self.time_value.config(text=f"{t:6.2f} s")
            self.velo_value.config(text=f"{vel} ft/s")
            self.alt_value.config(text=f"{alt} Feet")
            self.temp_value.config(text=f"{temp} °F")
            self.labelDrogue.config(bg="green" if 'D' in dma else 'red')
            self.labelMain.config(bg="green" if 'M' in dma else 'red')
            self.log.write(f"Packet # {self.packets}\n")
            self.log.write(f"T+: {t}\n")
            self.log.write(f"Altitude: {alt})\n")
            self.log.write(f"Velocity: {vel})\n")
            self.log.write(f"Temperature: {temp})\n")
            self.log.write(f"DMA Status: {dma}\n")
            self.log.write("----------------------------\n")
            self.packets += 1

        self.parent.after(100, self.update_values)
        

if __name__ == "__main__":
    parent = tk.Tk()
    app = DashboardApp(parent, getCOMS())
    parent.mainloop()