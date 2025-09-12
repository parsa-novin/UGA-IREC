import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# CHANGE THESE IF NEEDED
FrontalArea = 0.1486 # frontal area of the rocket in feet squared
BurnoutMass = 50 # mass of the rocket with the burnt out motor, in pounds-mass
AirTemp = 20 # degrees in celsius

GPSData = pd.read_csv('KJ4SAE01517_05-03-2025_16_57_21.csv')
# reading csv for GPS

# reading csv for primary altimeter
StartTime = GPSData['UNIXTIME'].iloc[0]

TimeModifier = 0.5  
GPSData['UNIXTIME'] = (GPSData['UNIXTIME'] - (StartTime + TimeModifier)).round(1)
# subtract a value from the UNIX time to yield a more readable value (zeroing time).
# The starting UNIX time is 1744577953.7, I like 1744577957.0 because I see the first movement immediately afterwards.
# THERE'S SO MUCH FLOATING POINT ERROR JESUS CHRIST
# One degree of latitude/longitude is 364,567.2 feet. 
# We are not taking the curvature of the earth into account since we are not going that far, all things considered.

# Getting starting latitude to correct for earth's curvature when doing longitude to distance conversion  
# (longitude distance becomes smaller the further north you are)
LAT_index = GPSData.columns.get_loc('LAT')
StartingLatitude = GPSData.iat[2, LAT_index]
CorrectionFactor = abs(np.cos(StartingLatitude))

# This correction factor will change depending on the starting latitude, 
# which will be nice for flights in different areas, like our real flight in Texas.

# HEY HEY HEY geopy.distance is a thing maybe check that out
GPSData['LAT'] = GPSData['LAT'] - 34.05208 
GPSData['LON'] = GPSData['LON'] - -80.41969 # Zeroing launch pad lat and long

GPSData['DeltaT'] = GPSData['UNIXTIME'].diff() # calculate delta t of between entries. It changes from 0.1s to 1s after apogee detection.
# GPSData['Calc_VERTV'] = GPSData['Altitude AGL']/GPSData['DeltaT'] # manually claculate vertical velocity to see if it's legit

# plt.plot(GPSData['UNIXTIME'], GPSData['VERTV'], label='Given Vertical Velocity')
# plt.plot(GPSData['UNIXTIME'], GPSData['VERTV'], label='Calculated Vertical Velocity')
# plt.legend()
# plt.show()
# Lines overlap, verified that the given vertical velocity is done this way.

# converting lat/long difference to horizontal distance from pad in feet 
GPSData['EWFeet'] = GPSData['LON'] * 364567.2 * CorrectionFactor
GPSData['NSFeet'] = GPSData['LAT'] * 364567.2   

# Calculating horizontal velocities
GPSData['EWVel'] = GPSData['EWFeet'].diff() / GPSData['DeltaT']
GPSData['NSVel'] = GPSData['NSFeet'].diff() / GPSData['DeltaT']

GPSData['EWVelSmooth'] = GPSData['EWVel'].rolling(window=7, center=True).mean()
GPSData['NSVelSmooth'] = GPSData['NSVel'].rolling(window=7, center=True).mean()

# Calculate angle from vertical (zenith) based off the velocity readout
GPSData['Zenith'] = 90 - ((np.arctan(GPSData['VERTV']/GPSData['HORZV'])) * 180/np.pi)
# Calculating total velocity
GPSData['TotalV'] = np.sqrt(GPSData['HORZV']**2 + GPSData['VERTV']**2)
# Calculating GPS and Altimiter vertical acceleration (and GPS horizontal acceleration)
GPSData['VERTA'] = GPSData['VERTV'].diff() / GPSData['DeltaT']
GPSData['HORZA'] = GPSData['HORZV'].diff() / GPSData['DeltaT']

# Smoothing them (they be roughhhhhhh)
GPSData['VERTASmooth'] = GPSData['VERTA'].rolling(window=7, center=True).mean()
GPSData['HORZASmooth'] = GPSData['HORZA'].rolling(window=7, center=True).mean()

# I need to create total GPS acceleration, then add 32.2 to the vertical component 
# so I get total acceleration due to just drag
GPSData['VERTA_NO_GRAVITY'] = GPSData['VERTASmooth'] + 32.2
GPSData['DragA'] = np.sqrt(GPSData['HORZA']**2 + GPSData['VERTA_NO_GRAVITY']**2)
GPSData['DragA_Smooth'] = GPSData['DragA'].rolling(window=17, center=True).mean()


# Saving to CSV's for diagnostic purposes
GPSData.to_csv('pythonmoddedgpsdata.csv')

# # # # # # # # # # # # # # # # # # # # # # # # # #
#        WELCOME TO THE                           #
#                           PLOT ZONE             #
# # # # # # # # # # # # # # # # # # # # # # # # # #

plt.close('all') # close all currently open figures
fig1 = plt.figure(1, figsize=(10.,10.))
ax1 = fig1.add_subplot(2,1,1)
ax2 = fig1.add_subplot(2,1,2)

ax1.title.set_text('Horizontal Velocities (ft/s)')
ax1.plot(GPSData['UNIXTIME'], GPSData['EWVelSmooth'], label='East-West')
ax1.plot(GPSData['UNIXTIME'], GPSData['NSVelSmooth'], label='North-South')
ax1.plot(GPSData['UNIXTIME'], GPSData['HORZV'], label='Combined Horizontal')
# ax1.plot(GPSData['UNIXTIME'], GPSData['CalcHorVelSmooth'], label='Calculated Horizontal Velocity')
ax1.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax1.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax1.legend()
ax1.grid()

ax2.title.set_text('Distance from Pad (ft)')
ax2.plot(GPSData['UNIXTIME'], GPSData['EWFeet'], label='East-West')
ax2.plot(GPSData['UNIXTIME'], GPSData['NSFeet'], label='North-South')
ax2.plot(GPSData['UNIXTIME'], GPSData['Distance (feet)'], label='Total')
ax2.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax2.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax2.legend()
ax2.grid()

fig1.subplots_adjust(hspace=0.5) # fixing title overlapping axis label and numbers
fig1.savefig("Figure 1.png")

# Old deleted plot demonstrates that my calculated horizontal velocity 
# is a little different from what the GPS recorded.
# So, we'll just use that instead. 

# FIGURE 2: Altitude and Velocity
fig2 = plt.figure(2, figsize=(10.,10.))
ax1 = fig2.add_subplot(2,1,1)
ax2 = fig2.add_subplot(2,1,2)

ax1.title.set_text('Altitude Above Ground Level (AGL) (ft)')
ax1.plot(GPSData['UNIXTIME'], GPSData['Altitude AGL'], label='GPS')
ax1.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax1.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax1.legend()
ax1.grid()

ax2.title.set_text('Vertical Velocity (ft/s)')
ax2.plot(GPSData['UNIXTIME'], GPSData['VERTV'], label='GPS')
# ax2.plot(GPSData['UNIXTIME'], GPSData['TotalV'], label="Total Velocity")
ax2.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax2.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax2.legend()
ax2.grid()

fig2.subplots_adjust(hspace=0.45) # fixing title overlapping axis label and numbers
fig2.savefig("Figure 2.png")

# FIGURE 3: ZOOMED IN Altitude and Velocity
fig3 = plt.figure(3, figsize=(10.,10.))
ax1 = fig3.add_subplot(2,1,1)
ax2 = fig3.add_subplot(2,1,2)

ax1.title.set_text('AGL Zoomed (ft)')
ax1.plot(GPSData['UNIXTIME'], GPSData['Altitude AGL'], label='GPS')
ax1.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax1.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout

ax1.set_xlim([0, 30])
ax1.legend()
ax1.grid()

ax2.title.set_text('Vertical Velocity Zoomed (ft/s)')
ax2.plot(GPSData['UNIXTIME'], GPSData['VERTV'], label='GPS')
# ax2.plot(PrimAltData['Time'], PrimAltData['Velocity'], label='Altimeter')
ax2.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax2.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax2.set_xlim([0, 30])
ax2.legend()
ax2.grid()

fig3.subplots_adjust(hspace=0.45) # fixing title overlapping axis label and numbers
fig3.savefig("Figure 3.png")

# FIGURE 4: Velocity Vector Angle & Acceleration
fig4 = plt.figure(4, figsize=(10.,10.))
ax1 = fig4.add_subplot(2,1,1)
ax2 = fig4.add_subplot(2,1,2)

ax1.title.set_text('Velocity Vector Angle (From GPS Velocity Data)')
ax1.plot(GPSData['UNIXTIME'], GPSData['Zenith'], label='Degrees from Vertical')
ax1.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax1.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax1.legend()
ax1.grid()
ax1.invert_yaxis() # flip the vertical axis
ax1.set_xlim(0,20)
ax1.set_yticks([0, 45, 90, 135])
ax1.set_yticklabels(['0', '45', '90', '45'])

ax2.title.set_text('Vertical Acceleration (ft/s^2)')
ax2.plot(GPSData['UNIXTIME'], GPSData['VERTASmooth'], label='GPS')
# ax2.plot(PrimAltData['Time'], PrimAltData['VERTASmooth'], label='Altimeter')
ax2.axvline(x=16.4, color='k', linestyle='--', label='Apogee') # line denoting apogee
ax2.axvline(x=3.2, color='r', linestyle='--', label='Motor Burnout') # line denoting motor burnout
ax2.legend()
ax2.grid()
ax2.set_ylim([-100, 300])
ax2.set_xlim([0, 20])
# UGLY ASS DATA AINT NO WAY WE CAN GET A DRAG COEFFICIENT FROM THIS
# BUT IM GONNA GIVE IT A GO
fig4.subplots_adjust(hspace=0.45) # fixing title overlapping axis label and numbers
fig4.savefig("Figure 4.png")

plt.show()