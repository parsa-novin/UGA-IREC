import tkinter as tk
from tkinter import ttk
import random
import time

import serial.tools.list_ports

def getCOMS():

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
    return portVar

class DashboardApp:
    def __init__(self, parent, portVar):
        self.parent = parent
        self.portVar = portVar
        #fill the screen
        parent.state('zoomed')

        self.parent.title("UGA IREC Telemetry Dashboard")

        # Create a Frame to hold the dashboard widgets
        self.dashboard_frame = ttk.Frame(parent)
        self.dashboard_frame.grid(sticky="nsew")

        #make labels updatable
        self.time_value = tk.Label(self.dashboard_frame, text= "UNCHANGE" + " Seconds", font=("Arial Black", 12))

        # Create and place widgets on the dashboard using the Grid geometry manager
        self.create_widgets(portVar)

    def create_widgets(self, portVar):
        # Create and place labels
        labelAlt = tk.Label(self.dashboard_frame, text="X", bg = "light grey", font=("Arial Black", 16))
        labelAlt.grid(row=0, column=0, padx = 0, pady = 0, sticky = "nsew")

        labelVelo = tk.Label(self.dashboard_frame, text="Y", bg = "light grey", font=("Arial Black", 16))
        labelVelo.grid(row=1, column=0, sticky = "nsew")

        labelAcc = tk.Label(self.dashboard_frame, text="Z", bg = "light grey", font=("Arial Black", 16))
        labelAcc.grid(row=2, column=0, sticky = "nsew")

        labelTemp = tk.Label(self.dashboard_frame, text= portVar , bg = "light grey", font=("Arial Black", 16))
        labelTemp.grid(row=3, column=0, sticky = "nsew")

        labelTime = tk.Label(self.dashboard_frame, text="Time Since Liftoff", bg = "light grey", font=("Arial Black", 16))
        labelTime.grid(row=4, column=0, sticky = "nsew")


        label4 = tk.Label(self.dashboard_frame, text="Parachute Status", bg = "light grey", font=("Arial Black", 16))
        label4.grid(row=0, column=4, rowspan= 1, sticky ="nsew")


        labelAngAtt = tk.Label(self.dashboard_frame, text="Angle of Attack", bg = "light grey", font=("Arial Black", 16))
        labelAngAtt.grid(row=3, column=4, padx = 0, sticky = "nsew")

        labelRadVelo = tk.Label(self.dashboard_frame, text="Radial Velocity", bg = "light grey", font=("Arial Black", 16))
        labelRadVelo.grid(row=4, column=4, sticky = "nsew")



        # Create and place statistics widgets (placeholders)
        alt_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Feet", font=("Arial Black", 12))
        alt_value.grid(row=0, column=0, sticky ="s")\
        
        velo_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " ft/s", font=("Arial Black", 12))
        velo_value.grid(row=1, column=0, sticky ="s")

        acc_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " ft/s^2", font=("Arial Black", 12))
        acc_value.grid(row=2, column=0, sticky ="s")

        temp_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " F", font=("Arial Black", 12))
        temp_value.grid(row=3, column=0, sticky ="s")
        
        #time_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Seconds", font=("Arial Black", 12))
        self.time_value.grid(row=4, column=0, sticky ="s")

        angAtt_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " Degrees", font=("Arial Black", 12))
        angAtt_value.grid(row=3, column=4, sticky ="s")

        radVelo_value = tk.Label(self.dashboard_frame, text= str(random.randint(1, 100)) + " deg/sec", font=("Arial Black", 12))
        radVelo_value.grid(row=4, column=4, sticky ="s")

        # Create and place graph widgets (placeholders)
        # You can use Matplotlib or other libraries for more advanced graphs
        footage = tk.Canvas(self.dashboard_frame, bg="white", width=400, height=400)
        footage.grid(row=0, column=1, columnspan= 3, rowspan =3, sticky="nsew")

        sponsor = tk.Canvas(self.dashboard_frame, bg="grey", width=400, height=400)
        sponsor.grid(row=3, column=1, columnspan= 3, sticky="nsew")

        miniRoc = tk.Canvas(self.dashboard_frame, bg="red", width=400, height=400)
        miniRoc.grid(row=4, column=1, columnspan= 3, sticky="nsew")

        # Configure grid layout weights to make the dashboard responsive
        for col in range(5):
            self.dashboard_frame.columnconfigure(col, weight=1, uniform ="equal")
        for row in range(5):
            self.dashboard_frame.rowconfigure(row, weight=1, uniform ="equal")

        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(0, weight=1)

if __name__ == "__main__":
    parent = tk.Tk()
    app = DashboardApp(parent, getCOMS())

    i = 10
    while i < 0:
        app.time_value.config(text = f"count: {i}")
        parent.update()
        i -= 1
        time.sleep(0.1)
    parent.mainloop()