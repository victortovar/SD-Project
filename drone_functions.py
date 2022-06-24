## Victor Tovar
## Drone Functions needed for flight written by Hayden Lotspeich

# Imports
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
#import exceptions
import math
import argparse

# Function to connect the raspberry pi to the pixhawk (takes in no arguments and returns vehicle object)
def connectMyCopter(connection_string, baud_rate):
    vehicle = connect(connection_string, baud=baud_rate, wait_ready = True)
    return vehicle

# Function to arm drone and make it take off to a specified height (in meters)
def arm_and_takeoff(vehicle, aTargetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    
    # Switch vehicle to GUIDED mode and wait for change
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    
    # Arm vehicle once GUIDED mode is confirmed
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed...")
        time.sleep(1)

    vehicle.simple_takeoff(aTargetAltitude)

    # Print current altitude and stop once we are within 95% of the target altitude
    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= (aTargetAltitude * 0.95):
            break
        time.sleep(1)

    print("Target altitude reached.")
    return vehicle

## Returns the distance in meters between two location points
def get_distance_meters(aLocation1, aLocation2):
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.112195e5