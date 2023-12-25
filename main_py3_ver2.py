from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy import distance

# Set up option parsing to get connection string
import argparse
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


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,        # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,           # confirmation
        heading,     # param 1, yaw in degrees
        0,           # param 2, yaw speed deg/s
        1,           # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)     # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print('Sent "yaw" message')


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

d = {'1': 'Roll 1', '2': 'Pitch 2', '3': 'Throttle 3', '4': 'Yaw 4', '5': 'Flight mode 5'}
def print_channels(vehicle):
    for k, v in d.items():
        print(v, ': ', vehicle.channels[k], end='  ')
    print()

height = 15
arm_and_takeoff(height)

airspeed = 20
print("Set default/target airspeed to %i" %airspeed)
vehicle.airspeed = 20

# point_B = LocationGlobalRelative(50.443326, 30.448078, height)
point_B = LocationGlobalRelative(50.450787, 30.460341, height)
vehicle.simple_goto(point_B)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)
print_channels(vehicle); time.sleep(0.1)

# print("Setting AltHold mode")
# print("\nMode:\t", vehicle.mode.name, "\n")
# # print("\nChannels BEFORE alt hold\n", vehicle.channels, "\n")
# time.sleep(5)
# vehicle.mode = VehicleMode("ALT_HOLD")
# vehicle.channels.overrides['3'] = 1500
# time.sleep(5)
# # vehicle.simple_goto(point_B)
# # time.sleep(1)
# print("\nMode:\t", vehicle.mode.name, "\n")
# vehicle.channels.overrides['2'] = 1500
# print("\nChannels AFTER alt hold\n", vehicle.channels, "\n")

curr_p = vehicle.location.global_frame  # we need just lat, lon
dst = distance.distance((curr_p.lat, curr_p.lon), (point_B.lat, point_B.lon)).m
while dst > 0.5:
    print("Distance to target: ", dst)
    print_channels(vehicle)
    time.sleep(1)
    curr_p = vehicle.location.global_frame
    dst = distance.distance((curr_p.lat, curr_p.lon), (point_B.lat, point_B.lon)).m

# time.sleep(2)
# vehicle.mode = VehicleMode("ALT_HOLD")
# vehicle.channels.overrides['3'] = 1500
# time.sleep(1)
# # vehicle.simple_goto(point_B)
# # time.sleep(1)
time.sleep(4)
# vehicle.mode = VehicleMode("STABILIZE")
vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.channels.overrides['3'] = 1500
vehicle.flush()
print("\nMode:\t", vehicle.mode.name, "\n")
# print("\nMode:\t", vehicle.mode.name, "\n")
time.sleep(2)
print("Setting YAW to 350")
vehicle.flush()
condition_yaw(350)
time.sleep(4)
print("Setting YAW to 250")
condition_yaw(250)


if sitl:
    print("Shutting down the simulator")
    sitl.stop()