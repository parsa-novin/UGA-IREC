import rocketpy
import datetime

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
print("hello world")
# change to your path
#df = pd.read_csv("C:/Users/andre/Downloads/dragCoeff_vs_mach_noNulls.csv")
df = pd.read_csv("mach_cdd.csv") #USE THIS ONE FOR SIMS
#df = pd.read_csv("HELP.csv") #USE THIS ONE FOR ORK

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
    radius= 0.078359, # meters
    mass=23.243, # kg
    inertia=(7.15, 7.15, 0.095), #kgm^2
    power_off_drag=power_off_drag_curve,
    power_on_drag=power_on_drag_curve,
    center_of_mass_without_motor=1.255014, #49.41in
    coordinate_system_orientation="nose_to_tail",
)

#switch to your file path
#M1939 = rocketpy.GenericMotor.load_from_eng_file("D:/AeroTech_M1939W.eng")
M1939 = rocketpy.GenericMotor.load_from_eng_file("m3400.eng") #not actually 1939 im just too lazy to change the motor name
from rocketpy.motors import GenericMotor

# From ThrustCurve (units converted to meters/kg)
diameter_m = 98e-3          # 98 mm :contentReference[oaicite:4]{index=4}
length_m   = 702e-3         # 702 mm :contentReference[oaicite:5]{index=5}
burn_time  = 2.9            # s :contentReference[oaicite:6]{index=6}

prop_mass  = 4.452          # kg (4452 g) :contentReference[oaicite:7]{index=7}

# Dry mass choice:
# - Cesaroni "Burnout Weight" = 3342 g (post-burn hardware + whatever their definition includes)
dry_mass   = 3.342          # kg :contentReference[oaicite:8]{index=8}

# Geometry for GenericMotor:
# chamber_radius: use motor radius (98 mm / 2)
chamber_radius = diameter_m / 2

# RocketPy default assumption when missing: nozzle_radius = 0.85 * chamber_radius :contentReference[oaicite:9]{index=9}
nozzle_radius = 0.85 * chamber_radius

motor = GenericMotor(
    thrust_source="m3400.eng",   # your .eng file path
    burn_time=burn_time,
    chamber_radius=chamber_radius,
    chamber_height=length_m,
    chamber_position=0.0,
    propellant_initial_mass=prop_mass,
    nozzle_radius=nozzle_radius,
    dry_mass=dry_mass,

    # NOT on the thrustcurve page -> placeholders (you should replace if you have real data)
    center_of_dry_mass_position=1.0,
    dry_inertia=(0.0, 0.0, 0.0),

    nozzle_position=0.0,
    reshape_thrust_curve=False,
    interpolation_method="linear",
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

motor.info()

cloud_test.add_motor(M1939, position = 2.1971)
print("helloworld2")
nose_cone = cloud_test.add_nose(
    length = 0.762, # meters
    kind = 'ogive',
    position = 0, # meters
    base_radius = 0.078359
)

tailCone = cloud_test.add_tail(
    top_radius = 0.156718/2, # meters
    bottom_radius = 0.1143/2, # meters
    length = 0.127, # meters
    position = 2.0828 # meters
)

fin_set = cloud_test.add_trapezoidal_fins(
    n=3,
    root_chord=0.2032,
    tip_chord=0.2032/2,
    span=0.1778,
    position=1.8796
)

env = rocketpy.Environment(
    latitude = 31.02403,         # degrees north
    longitude = -103.66157,      # degrees west
    elevation = 198.73           # meters above sea level
)
print("helloworld3")
# right now the date is just set to tmw
tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))  
env.set_atmospheric_model(type="Forecast", file="GFS")
print("Backend:", matplotlib.get_backend())
cloud_test.draw()
plt.show()
flight = rocketpy.Flight(
    rocket=cloud_test,
    environment=env,
    rail_length=5.2,   # meters, length of the launch rail (i just guessed)
    inclination=87.5,    # degrees (from horizontal, so ~vertical)
    heading=0          # degrees azimuth (0 = north, clockwise)
)

# theres other functions you can also use if you want other data
flight.info()
