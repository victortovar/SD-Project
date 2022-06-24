# Victor Tovar
# UTA CSE drone team


# This program takes the lidar info and creates a visualization of vectors that display the path
# the drone would take upon encountering obstacles in its path

import numpy as np
from matplotlib import pyplot as plt
import os
import sys
import time
from rplidar import RPLidar
from math import cos, sin, pi, sqrt, radians
import matplotlib.animation as animation

def connect_lidar():
    PORT_NAME = '/dev/tty.usbserial-0001'
    lidar = RPLidar(PORT_NAME)
    #lidar.stop_motor()
    #health = lidar.get_health()
    #print("LIDAR health: "% health)
    return lidar

def avoidance(lidar, destination):
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    out_vector_x = 0.0
    out_vector_y = 0.0
    
    #scans_per_rotation = 400
    avoid = False
    ## max_correction_distance will be used to determine the maximum amount of distance the 
    ## drone can move in a specific direction when avoiding obstacles.
    ## Distance is given in meters
    max_correction_distance = 0.25
    x_arr = []
    y_arr = []
    try:
        ## enumerate allows referencing of the scan and the index of list of scans
        ## index will be used to reset accumulated vector after one rotation of the lidar scanner
        
        for i, scan in enumerate(lidar.iter_scans()):
            ## Reset the accumulated vectors
            
            if( (i % 11) == 0):
                out_vector_x = 0.0
                out_vector_y = 0.0
                x_arr=[]
                y_arr=[]
                #vehicle.simple_goto(destination)

            for(_, angle, distance) in scan:
                ## ignore anything outside of the range between the min and max in millimeters
                max = 2000.0
                min = 130.0
                ## k is the scaling factor used for the repulsive potential function
                k = 0.5
                x_arr.append(distance*sin(radians(angle)))
                y_arr.append(distance*cos(radians(angle)))
                #print(type(x))
                # plt.clf()
                # plt.scatter(x_arr, y_arr)
                # plt.pause(0.0001)
                #plt.show() 
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
                curr_orientation = 0 #radians(vehicle.heading)
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
                out_vector_x = out_vector_x #/111139
                out_vector_y = out_vector_y #/111139
                print("x: %0.4f"% out_vector_x)
                print("y: %0.4f"% out_vector_y)
                # data = np.array([[2, 1], [-1, 2], [4, -1]])
                # origin = np.array([[0, 0, 0], [0, 0, 0]])
                # data = np.array([out_vector_x, out_vector_y])
                # origin = np.array([0, 0])
                # plt.quiver(*origin, data[0], data[1], color=['red'], scale=15)
                # plt.pause(0.1)
                #fig = plt.figure()
                #ani = animation.FuncAnimation(fig, change_vector, fargs=(out_vector_x, out_vector_y))
                #plt.show()
                ## need to get current location and set new destination
                #(curr_lat, curr_lon, _) = vehicle.location.global_frame
                ## sets new destination to avoid obstacles
                #curr_dest = LocationGlobalRelative(curr_lat + out_vector_x, curr_lon+ out_vector_y, vehicle.location.global_relative_frame.alt)
                #vehicle.simple_goto(curr_dest)
             
    ## control+c will terminate the program and will stop the lidar motor      
    except KeyboardInterrupt:
        lidar.stop_motor()
def change_vector(x, y):
    plt.quiver(0, 0, x, y, color=['red'], scale=15)


lidar = connect_lidar()
avoidance(lidar, (100, 100))

