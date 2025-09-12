import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv('telemetry.csv')

data['accel_total'] = np.sqrt(data['accel_x']**2 + data['accel_y']**2 + data['accel_z']**2) - 9.81

# ALL THIS IS IN METERS PER SECOND, RADIANS PER SECOND, AND MICROTESLAS

plt.close('all') # close all currently open figures

fig1 = plt.figure(1, figsize=(10.,10.))
ax1 = fig1.add_subplot(2,1,1)

ax1.title.set_text('accelerations')
ax1.plot(data['packet'], data['accel_x'], label='x accel')
ax1.plot(data['packet'], data['accel_y'], label='y accel')
ax1.plot(data['packet'], data['accel_z'], label='z accel')

ax1.legend()
ax1.grid()
# ax1.set_xlim([0, 20])
ax1.set_ylim([-50, 50])


fig2 = plt.figure(2, figsize=(10.,10.))
ax1 = fig2.add_subplot(2,1,1)
ax1.plot(data['packet'], data['accel_total'], label='total accel')


ax1.legend()
ax1.grid()
# ax1.set_xlim([0, 20])
# ax1.set_ylim([0, 10])


plt.show()