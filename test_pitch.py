from __future__ import print_function
import time
import argparse
import numpy as np

from math import degrees

from dronekit import connect, Vehicle, VehicleMode
from geographiclib.geodesic import Geodesic
import matplotlib.pyplot as plt

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(tgt_alt):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(tgt_alt)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # reaching required altitude
        if tgt_alt * 1.02 >= vehicle.location.global_relative_frame.alt >= tgt_alt * 0.98:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_curr_pos(vehicle: Vehicle):
    return vehicle.location.global_frame.lat, vehicle.location.global_frame.lon

def calculate_dist(lat1, lon1, lat2, lon2):
    """Обчислює відстань між точками з координатами `(lat1, lon1), (lat2, lon2)`"""

    geod = Geodesic.WGS84
    g = geod.Inverse(lat1, lon1, lat2, lon2)
    return g['s12']  # відстань
# ---------------------------- Головний скрипт --------------------------------------
# здійнятися на вказану висоту в метрах
height = 10
arm_and_takeoff(height)

# перейти в режим AltHold
vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.channels.overrides['3'] = 1500  # без цього дрон заоре носом
time.sleep(1)

time_interval = 10  # seconds
step = 0.25
# rc_deltas = [25, 50, 100, 150, 200, 225]#, 250, 300, 350, 400, 450, 500] # -500, 0]
rc_deltas = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 0]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [-40, -41, -42, -43, -44, -45, -46, -47, -48, -49, -50]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [-49, -50]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [50, 75, 100, 125, 150]#, 250, 300, 350, 400, 450, 500] # -500, 0]

x = np.linspace(0, len(rc_deltas) * time_interval, len(rc_deltas) * int(time_interval / step), endpoint=False)
pitches = np.zeros(len(x))
xx = np.zeros_like(pitches)

idx = 0
prev = curr = 0
for delta in rc_deltas:
    start = time.time()
    print(f'Setting pitch delta to\t{delta}; time = \t{start}')
    vehicle.channels.overrides['2'] = 1500 + delta
    while time.time() - start < time_interval:
        curr = get_curr_pos(vehicle)
        pitches[idx] = calculate_dist(*prev, *curr)
        prev = curr
        idx += 1
        time.sleep(step)

print(f'Simuliation finished; idx = {idx}, \tpitch len = {len(pitches)}')
vehicle.channels.overrides['2'] = 1500

delta_pitches = pitches[1:] - pitches[:-1]

pitches_v = delta_pitches / step  # velocities

pitches_a = (pitches_v[1:] - pitches_v[:-1]) / step  # acceleration

title = 'Pitch'
with open(f'test_{title.lower()}.npy', 'wb') as f:
    np.save(f, x)
    np.save(f, pitches)
    np.save(f, delta_pitches)
    np.save(f, pitches_v)
    np.save(f, pitches_a)

fig, (ax0, ax1, ax2) = plt.subplots(nrows=3)

ax0.plot(x, pitches, 'b')
ax0.set_title(title)
ax0.legend()

ax1.plot(x[:-1], pitches_v, 'g')
ax1.set_title(title + ' velocity')
ax1.legend()

ax2.plot(x[:-2], pitches_a, 'r')
ax2.set_title(title + " acceleration")
ax2.legend()


plt.tight_layout()
plt.show()
