import rocketpy
import datetime

#radius conversions
inch = 0.0254
r1 = 6 * inch / 2
r2 = 5.5 * inch / 2

import pandas as pd

# change to your path
df = pd.read_csv("C:/Users/andre/Downloads/dragCoeff_vs_mach_noNulls.csv")

# Weird chatgpt stuff (might be able to remove now but idk plus its not like it really changes much)
# Remove duplicate Mach numbers (keep the first occurrence)
df = df.drop_duplicates(subset='MachNumber___', keep='first')

# Ensure Mach 0 is included
if df['MachNumber___'].iloc[0] > 0:
    df = pd.concat([
        pd.DataFrame({'MachNumber___':[0], 'x_DragCoefficient___':[df['x_DragCoefficient___'].iloc[0]]}),
        df
    ], ignore_index=True)

# Make sure Mach is strictly increasing
df = df.sort_values('MachNumber___').reset_index(drop=True)



# convert to list of [Mach, Cd] for RocketPy
drag_table = df[['MachNumber___', 'x_DragCoefficient___']].values.tolist()
power_on_drag_curve = drag_table
power_off_drag_curve = drag_table

cloud_test = rocketpy.Rocket(
    radius= 0.06985, # meters
    mass=20.422, # kg
    inertia=(6.321, 6.321, 0.034), #kgm^2
    power_off_drag=power_off_drag_curve,
    power_on_drag=1,
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

#switch to your file path
M1939 = rocketpy.GenericMotor.load_from_eng_file("D:/AeroTech_M1939W.eng")

cloud_test.add_motor(M1939, position = -1.17)

nose_cone = cloud_test.add_nose(
    length = 0.61, # meters
    kind = 'conical',
    position = 1.18, # meters
    base_radius = r1
)

tailCone = cloud_test.add_tail(
    top_radius = 0.131, # meters
    bottom_radius = 0.102, # meters
    length = 0.102, # meters
    position = -1.12 # meters
)

fin_set = cloud_test.add_trapezoidal_fins(
    n=3,
    root_chord=0.230,
    tip_chord=0.080,
    span=0.163,
    position=-0.88956
)


# bottom two things are the transition
cloud_test.add_tail(
    top_radius = r1, 
    bottom_radius = r2,
    length = 0.102,
    position = .1,
    name = "transition 5.5→6 in"
)

cloud_test.add_tail(
    top_radius = r1,
    bottom_radius = r1 + .001,
    length = 0.48,               
    position = .5702,
    name = "6 in sustained body"
)

env = rocketpy.Environment(
    latitude = 32.00,         # degrees north
    longitude = -102.08,      # degrees west
    elevation = 875           # meters above sea level
)

# right now the date is just set to tmw
tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))  
env.set_atmospheric_model(type="Forecast", file="GFS")

flight = rocketpy.Flight(
    rocket=cloud_test,
    environment=env,
    rail_length=5.2,   # meters, length of the launch rail (i just guessed)
    inclination=85,    # degrees (from horizontal, so ~vertical)
    heading=0          # degrees azimuth (0 = north, clockwise)
)

# theres other functions you can also use if you want other data
flight.info()
