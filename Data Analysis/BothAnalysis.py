import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

GPSData = pd.read_csv('KJ4SAE01517_04-13-2025_12_59_18.csv')
# reading csv for GPS
# print(GPSData.head())

PrimAltData = pd.read_csv('Cloud ROSCO.csv')
# reading csv for primary altimeter
# print(PrimAltData.head())

GPSData['UNIXTIME'] = (GPSData['UNIXTIME'] - 1744577957).round(1)
# subtract a value from the UNIX time to yield a more readable value (zeroing time).
# The starting UNIX time is 1744577953.7, I like 1744577957.0 because I see the first movement immediately afterwards.
# THERE'S SO MUCH FLOATING POINT ERROR JESUS CHRIST
# One degree of latitude/longitude is 364,567.2 feet. 
# We are not taking the curvature of the earth into account since we are not going that far, all things considered.
# 
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

# Insert new rows that are feet from the launch pad north/west (or whatever orthogonal direction)
# ^ Do we super care about this besides for final landing point? ^ 
# Determine vertical heading 
# Determine vertical velocity with the ALT row - VERTV column exists
# Determine true velocity by doing cubic pythagorean theorem with ALT and orthogonal directions     
# 
GPSData['DeltaT'] = GPSData['UNIXTIME'].diff() # calculate delta t of between entries. It changes from 0.1s to 1s after apogee detection.
# GPSData['Calc_VERTV'] = GPSData['Altitude AGL']/GPSData['DeltaT'] # manually claculate vertical velocity to see if it's legit
# print(GPSData.head())

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
# GPSData['CalcHorVel'] = np.sqrt(GPSData['EWVel']**2 + GPSData['NSVel']**2)

GPSData['EWVelSmooth'] = GPSData['EWVel'].rolling(window=7, center=True).mean()
GPSData['NSVelSmooth'] = GPSData['NSVel'].rolling(window=7, center=True).mean()
# GPSData['CalcHorVelSmooth'] = GPSData['CalcHorVel'].rolling(window=5, center=True).mean()

# Calculate angle from vertical (zenith) based off the velocity readout
GPSData['Zenith'] = 90 - ((np.arctan(GPSData['VERTV']/GPSData['HORZV'])) * 180/np.pi)

GPSData.to_csv('pythonmoddedgpsdata.csv')

plt.close('all') # close all currently open figures

fig1 = plt.figure(1, figsize=(10.,10.))
ax1 = fig1.add_subplot(3,1,1)
ax2 = fig1.add_subplot(3,1,2)
ax3 = fig1.add_subplot(3,1,3)
# ax3 = fig.add_subplot(3,1,3)

ax1.plot(GPSData['UNIXTIME'], GPSData['EWVelSmooth'], label='East-West Velocity (ft/s)')
ax1.plot(GPSData['UNIXTIME'], GPSData['NSVelSmooth'], label='North-South Velocity (ft/s)')
ax1.plot(GPSData['UNIXTIME'], GPSData['HORZV'], label='Given Horizontal Velocity (ft/s)')
# ax1.plot(GPSData['UNIXTIME'], GPSData['CalcHorVelSmooth'], label='Calculated Horizontal Velocity')
ax1.legend()

ax2.plot(GPSData['UNIXTIME'], GPSData['VERTV'], label='Vertical Velocity (ft/s)')
ax2.legend()

ax3.plot(GPSData['UNIXTIME'], GPSData['EWFeet'], label='East-West Distance (ft)')
ax3.plot(GPSData['UNIXTIME'], GPSData['NSFeet'], label='North-South Distance (ft)')
ax3.plot(GPSData['UNIXTIME'], GPSData['Distance (feet)'], label='Total Horizontal Distance (ft)')
ax3.legend()


# HEY THE DISTANCE FOR LONGITUDE IS PROBABLY DIFFERENT BECAUSE WE'RE NOT AT THE EQUATOR
# Plot demonstrates that the horizontal velocity readout is a little different from what the GPS shows.
# So, we'll just use that instead. 

fig2 = plt.figure(2, figsize=(10.,10.))
ax1 = fig2.add_subplot(3,1,1)
ax2 = fig2.add_subplot(3,1,2)
ax3 = fig2.add_subplot(3,1,3)

ax1.plot(GPSData['UNIXTIME'], GPSData['Zenith'], label='Zenith Angle(degrees from vertical)')
plt.show()