## Victor Tovar
## CSE Drone Team 2022
## Task 3 - obstacle avoidance using GPS
## This script will connect the pi to the FC, connect the lidar to the pi, arm the drone,
## set its flight altitude, set its destination to the desired coordinate,
## and use the obstacle avoidance algorithm to avoid anything in its path towards its destination
## Code will use the functions that were written by Hayden Lotspeich for connecting and for flight


## To run this code, type "python Task3.py --connect /dev/ttyAMAO" (The end is an IP address and a port number, not sure which ones we will be using)

# Imports
from __future__ import print_function
from asyncio import subprocess
from obstacle_avoidance import *
from drone_functions import *
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
#import exceptions
import math
import argparse

## Sets the starting parameters for drone destination and altitude
lat = 0
lon = 0
alt = 4
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()
connection_string = args.connect
# Change rate depending on what rate we are able to connect the pi to the cube with
baud_rate = 57600
# port used to connect lidar to pi
PORT_NAME = '/dev/ttyUSB0'
destination = LocationGlobalRelative(lat, lon, alt)
vehicle = connectMyCopter(connection_string, baud_rate)
print("About to take off...")
vehicle.mode = VehicleMode("GUIDED")
vehicle = arm_and_takeoff(vehicle, alt)
vehicle.airspeed = 0.15 ## 15 cm/s OR about 0.5 ft/s
lidar = connect_lidar(PORT_NAME)
while get_distance_meters(destination, vehicle.location.global_frame) > 0.1:
    #vehicle.simple_goto(destination)
    avoidance(vehicle, lidar, destination)
vehicle.mode = VehicleMode("LAND")
lidar.stop_motor()
vehicle.close()

