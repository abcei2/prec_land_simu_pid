'''

	Synopsis: Script to run the control algorithm.
	Author: Nikhil Venkatesh
	Contact: mailto:nikv96@gmail.com

'''

# Dronekit Imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil

# Common Library Imports
from pid_utils.flight_assist import send_velocity
from pid_utils.position_vector import PositionVector
import pid_utils.pid as pid

# Python Imports
import math
import time
import argparse

# Global Variables
x_pid = pid.pid(0.1, 0.005, 0.1, 50)
y_pid = pid.pid(0.1, 0.005, 0.1, 50)
hfov = 80
hres = 640
vfov = 60
vres = 480
x_pre = 0
y_pre = 0


def pixels_per_meter(fov, res, alt):
    return ((alt * math.tan(math.radians(fov/2))) / (res/2))


def land(vehicle, target, attitude, location):
    if(vehicle.location.global_relative_frame.alt <= 0.5):
        vehicle.mode = VehicleMode('LAND')
    if(target is not None):
        move_to_target(vehicle, target, attitude, location)
    elif(vehicle.location.global_relative_frame.alt > 30):
        vehicle.mode = VehicleMode('LAND')
    else:
        send_velocity(vehicle, 0, 0, 0.25, 1)


def move_to_target(vehicle, target, attitude, location):
    x, y = target

    alt = vehicle.location.global_relative_frame.alt
    px_meter_x = pixels_per_meter(hfov, hres, alt)
    px_meter_y = pixels_per_meter(vfov, vres, alt)

    x *= px_meter_x
    y *= px_meter_y

    vx = x_pid.get_pid(x, 0.1)
    vy = y_pid.get_pid(y, 0.1)

    print("x = " + str(x), "vx = " + str(vx), "y = " + str(y),
          "vy = " + str(vy), "distance:", math.sqrt(x**2 + y**2),"alt", alt)
    vz = 0
    if alt < 3.5:
        min_distance=0.05
    else:
        min_distance=0.2


    if(math.sqrt(x**2 + y**2) < min_distance):
        vz = 0.25
    else:
        vz = 0
    send_velocity(vehicle, vy, vx, vz, 1)
