##Victor Tovar
##UTA CSE Team 2022

###need to install rplidar using pip:
###pip install rplidar-roboticia
###OR for python3:
###sudo pip3 install rplidar-roboticia

from curses.ascii import alt
import os
import sys
import time
from rplidar import RPLidar
import math
from math import cos, sin, pi, sqrt, radians
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException


## Connects to the port name for running the lidar sensor
def connect_lidar(PORT_NAME):
    # global lidar
    #PORT_NAME = '/dev/tty.usbserial-0001'
    lidar = RPLidar(PORT_NAME)
    # lidar.stop_motor()
    # health = lidar.get_health()
    # print("LIDAR health: "% health)
    return lidar

##iter_scans returns a list of three values in the form of 
# (quality(pulse strength), angle[0-360), distance(millimeters))
# need to pass the lidar sensor and the final destination that the drone needs to arrive to
def avoidance(vehicle, lidar, destination):
    out_vector_x = 0.0
    out_vector_y = 0.0
    #scans_per_rotation = 400
    avoid = False
    ## max_correction_distance will be used to determine the maximum amount of distance the 
    ## drone can move in a specific direction when avoiding obstacles.
    ## Distance is given in meters
    max_correction_distance = 0.25
    try:
        ## enumerate allows referencing of the scan and the index of list of scans
        ## index will be used to reset accumulated vector after one rotation of the lidar scanner
        for i, scan in enumerate(lidar.iter_scans()):
            ## Reset the accumulated vectors
            if( (i % 11) == 0):
                out_vector_x = 0.0
                out_vector_y = 0.0
                vehicle.simple_goto(destination)

            for(_, angle, distance) in scan:
                ## ignore anything outside of the range between the min and max in millimeters
                max = 2000.0
                min = 130.0
                ## k is the scaling factor used for the repulsive potential function
                k = 0.5
                if(distance < max and distance > min):  
                    # print("object in range, need to move")
                    # print("distance: %f" %distance)
                    # print("angle (in radians): %f" %radians(angle))
                    avoid = True
                    ## converts angle to radians as the trig functions take in angle in radians
                    ## after it accounts for the lidar being upside down
                    angle_in_rad = radians(360 - angle)
                    ## converts distance to meters
                    distance = distance/1000
                    x = cos(angle_in_rad)
                    y = sin(angle_in_rad)
                    ## function that emulates repulsive potential
                    r = -0.5*k*pow(((1/distance) - 1/(max/1000)), 2)

                    ## calculates the vector that drone will use to avoid obstacle
                    out_vector_x = out_vector_x + x*r
                    out_vector_y = out_vector_y + y*r

                ## gets current orientation in radians
                curr_orientation = radians(vehicle.heading)
                out_vector_x = out_vector_x*cos(curr_orientation) - out_vector_y*sin(curr_orientation)
                out_vector_y = out_vector_x*sin(curr_orientation) + out_vector_y*cos(curr_orientation)

                if(avoid):
                    c = sqrt(pow(out_vector_x, 2) + pow(out_vector_y, 2))
                    ## conditional statement determines how much the drone is allowed to moved in a specific direction
                    ## to avoid an obstacle and normalizes the vectors if it is in range
                    if(c > max_correction_distance):
                        out_vector_x = max_correction_distance*(out_vector_x/c)
                        out_vector_y = max_correction_distance*(out_vector_y/c)
                ## latitude - north south
                ## longitude - east west
                ### 111139 degrees to meters
                out_vector_x = out_vector_x/111139
                out_vector_y = out_vector_y/111139

                ## need to get current location and set new destination
                curr_location = vehicle.location.global_frame
                ## sets new destination to avoid obstacles
                curr_dest = LocationGlobalRelative(curr_location.lat + out_vector_x, curr_location.lon + out_vector_y, vehicle.location.global_relative_frame.alt)
                vehicle.simple_goto(curr_dest)
                
    ## control+c will terminate the program and will stop the lidar motor      
    except KeyboardInterrupt:
        lidar.stop_motor()
