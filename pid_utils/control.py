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
x_pid = pid.pid(0.2, 0.005, 0.1, 50)
y_pid = pid.pid(0.2, 0.005, 0.1, 50)
z_pid = pid.pid(0.05, 0.005, 0.1, 50)
hfov = 80
hres = 640
vfov = 60
vres = 480
x_pre = 0
y_pre = 0


def pixels_per_meter(fov, res, alt):
    return ((alt * math.tan(math.radians(fov/2))) / (res/2))


def land(vehicle, target, attitude, location,forensic_message):
    
    if vehicle.mode ==  VehicleMode('RTL'):
        vehicle.mode = VehicleMode('GUIDED')

        
    if(vehicle.location.global_relative_frame.alt <= 0.3):
        vehicle.mode = VehicleMode('LAND')
        
    if(target is not None):
        vx, vy, vz, dist_error = move_to_target(vehicle, target, attitude, location)
        forensic_message["control_stats"]["vx"]=vx
        forensic_message["control_stats"]["vy"]=vy
        forensic_message["control_stats"]["vz"]=vz
        forensic_message["marker_stats"]["dist_error"]=dist_error

    else:
        vx=0
        vy=0
        vz=0.25
        forensic_message["control_stats"]["vx"]=vx
        forensic_message["control_stats"]["vy"]=vy
        forensic_message["control_stats"]["vz"]=vz
        send_velocity(vehicle, vy, vx, vz, 1)
    


def move_to_target(vehicle, target, attitude, location):
    x, y = target

    alt = vehicle.location.global_relative_frame.alt
    px_meter_x = pixels_per_meter(hfov, hres, alt)
    px_meter_y = pixels_per_meter(vfov, vres, alt)

    x *= px_meter_x
    y *= px_meter_y
    z = alt
    vx = x_pid.get_pid(x, 0.1)
    vy = y_pid.get_pid(y, 0.1)

    dist_error = math.sqrt(x**2 + y**2+z**2)
    vz = z_pid.get_pid(dist_error, 0.1)
    #print("alt = " + str(alt), "vz = " + str(vz), "distance:", dist_error,"alt", alt)
          
    send_velocity(vehicle, vy, vx, vz, 1)
    return vx, vy, vz, dist_error
