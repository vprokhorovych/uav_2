from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
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


# def condition_yaw(heading, relative=False):
#     if relative:
#         is_relative = 1 #yaw relative to direction of travel
#     else:
#         is_relative = 0 #yaw is an absolute angle
#     # create the CONDITION_YAW command using command_long_encode()
#     msg = vehicle.message_factory.command_long_encode(
#         0, 0,        # target system, target component
#         mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
#         0,           # confirmation
#         heading,     # param 1, yaw in degrees
#         0,           # param 2, yaw speed deg/s
#         1,           # param 3, direction -1 ccw, 1 cw
#         is_relative, # param 4, relative offset 1, absolute angle 0
#         0, 0, 0)     # param 5 ~ 7 not used
#     # send command to vehicle
#     vehicle.send_mavlink(msg)
#     print('Sent "yaw" message')


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

from geographiclib.geodesic import Geodesic
from math import degrees

def calculate_dist_azimuth(lat1, lon1, lat2, lon2):
    geod = Geodesic.WGS84
    g = geod.Inverse(lat1, lon1, lat2, lon2)
    return g['s12'], g['azi1']

def get_pitch(dist, dist_error, print_msg=True):
    if dist < dist_error:
        delta = 0
    elif dist < 20:
        delta = 55
    elif dist < 40:
        delta = 70
    elif dist < 40:
        delta = 120
    else:
        delta = 1000
    res = 1500 - delta
    if print_msg:
        print("Curr dist: ", dist) #, "\tdelta: ", delta, "\tres: ", res)
    return res

def get_azim(azim):
    return azim + (360 if -180 <= azim < 0 else 0)

def get_yaw(delta_azim, delta_error):
    abs_delta_azim = abs(delta_azim)
    sgn = 1 if delta_azim > 0 else (-1 if delta_azim < 0 else 0)
    
    if abs_delta_azim < delta_error:
        delta = 0
    elif abs_delta_azim < 15:
        delta = 47 if sgn == 1 else 52
    elif abs_delta_azim < 80:
        delta = 65
    else:
        delta = 100
    res = 1500 - sgn * delta
    print(f"Delta azim: {delta_azim}, res: {res}")
    return res

def move_to(vehicle: Vehicle, lat, lon, dist_error=0.5):
    overrides = vehicle.channels.overrides
    time_step = 0.25

    c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # current
    dist, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)
    print("Init dist: ", dist)
    azim = get_azim(azim)
    curr_azim = get_azim(degrees(vehicle.attitude.yaw))
    while dist >= dist_error:
        overrides['2'] = get_pitch(dist, dist_error)
        overrides['4'] = get_yaw(curr_azim - azim, 1)
        time.sleep(time_step)

        c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # current
        dist, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)
        azim = get_azim(azim)
        curr_azim = get_azim(degrees(vehicle.attitude.yaw))

        while abs(curr_azim - azim) > 1.5 and 5 > dist >= dist_error:
            overrides['2'] = get_pitch(0, dist_error, False)
            time.sleep(time_step)
            overrides['4'] = get_yaw(curr_azim - azim, 1)
            time.sleep(time_step)

            c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # current
            dist, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)
            azim = get_azim(azim)
            curr_azim = get_azim(degrees(vehicle.attitude.yaw))

    print("\nArrived; curr dist: ", dist)
    overrides['2'] = get_pitch(0, dist_error, False)
    overrides['4'] = get_yaw(0, 1)
    

def rotate(vehicle: Vehicle, lat, lon, azim_error=1):
    overrides = vehicle.channels.overrides
    time_step = 0.25

    c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # current
    _, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)
    azim = get_azim(azim)
    print("Desired azim: ", azim)
    curr_azim = get_azim(degrees(vehicle.attitude.yaw))
    delta_azim = curr_azim - azim
    while abs(delta_azim) >= azim_error:
        overrides['4'] = get_yaw(delta_azim, azim_error)
        time.sleep(time_step)

        c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # current
        _, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)
        azim = get_azim(azim)
        curr_azim = get_azim(degrees(vehicle.attitude.yaw))
        delta_azim = curr_azim - azim

    print("Rotated; curr azim: ", curr_azim, "\tazim error: ", delta_azim)
    overrides['4'] = get_yaw(0, azim_error)


def rotate_yaw(vehicle: Vehicle, azim, azim_error=1):
    overrides = vehicle.channels.overrides
    time_step = 0.25

    print("Desired azim: ", azim)
    curr_azim = get_azim(degrees(vehicle.attitude.yaw))
    delta_azim = curr_azim - azim
    while abs(delta_azim) >= azim_error:
        overrides['4'] = get_yaw(delta_azim, azim_error)
        time.sleep(time_step)

        curr_azim = get_azim(degrees(vehicle.attitude.yaw))
        delta_azim = curr_azim - azim

    print("Rotated; curr azim: ", curr_azim, "\tazim error: ", delta_azim)
    overrides['4'] = get_yaw(0, azim_error)


height = 100
arm_and_takeoff(height)

vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.channels.overrides['3'] = 1500
time.sleep(1)

rotate(vehicle, 50.443326, 30.448078)
time.sleep(5)
move_to(vehicle, 50.443326, 30.448078)
print("\nArrived; setting yaw to 350\n")
rotate_yaw(vehicle, 350)
time.sleep(5)


# point_B = LocationGlobalRelative(50.443326, 30.448078, height)
# vehicle.simple_goto(point_B)

# print("Setting AltHold mode")
# airspeed = 20
# rotate(vehicle, 50.450787, 30.460341)
# move_to(vehicle, 50.450787, 30.460341)
# print("Set default/target airspeed to %i" %airspeed)
# vehicle.airspeed = 20
# print("\nMode:\t", vehicle.mode.name, "\n")
# # print("\nChannels BEFORE alt hold\n", vehicle.channels, "\n")
# time.sleep(5)
# # vehicle.simple_goto(point_B)
# # time.sleep(1)
# print("\nMode:\t", vehicle.mode.name, "\n")
# vehicle.channels.overrides['2'] = 1500
# print("\nChannels AFTER alt hold\n", vehicle.channels, "\n")

# print("Setting YAW to 350")
# condition_yaw(350)

if sitl:
    print("Shutting down the simulator")
    sitl.stop()