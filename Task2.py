# Name: Hayden Lotspeich
# Genius, greatest, awesome, hard working, humble, kind, "the best" - Albert Einstein

# Victor Tovar
# Excellent food purchaser, kind, friendly, charitable

# Infinitesimally microscopic contribution by Tyler Westbrook
# A person, dad, IT guy

# Drone Code Video 2

# This script will connect the pi to the pixhawk, arm the drone, raise it to a set altitude, then land the drone

# To turn code into executable, run "chmod +x dronecodevid2.py"

# To run this code, type "python Task2.py --connect /dev/ttyAMAO" (The end is an IP address and a port number, not sure which ones we will be using)

# Imports
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
import subprocess
import sys
import threading
import calendar
import ast
import cv2

CAMERAMATRIX = [[4.108219415216477159e+02,0.000000000000000000e+00,3.209051905619184595e+02],
[0.000000000000000000e+00,4.095246480960400390e+02,2.072528488488969742e+02],
[0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]]
CAMERADISTORTION = [2.294472942790989900e-02,-1.900828315928471401e-01,9.216477252615798768e-03,1.811202921724780306e-03,1.465316072432057137e-01]

def outputReader(process, outqueue):
	packet = []
	while True:
		line = process.stdout.readline()
		outqueue.put(line)

#def arucoLand()

def get_distance_meters(aLocation1, aLocation2):
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.112195e5

# Function to connect the raspberry pi to the pixhawk (takes in no arguments and returns vehicle object)
def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	# Change rate depending on what rate we are able to connect the pi to the cube with
	baud_rate = 57600

	vehicle = connect(connection_string, baud=baud_rate, wait_ready = True)
	return vehicle

# Function to arm drone and make it take off to a specified height (in meters)
def arm_and_takeoff(aTargetAltitude):
	while not vehicle.is_armable:
		print("Waiting for vehicle to become armable...")
		time.sleep(1)
	
	# Switch vehicle to GUIDED mode and wait for change
	vehicle.mode = VehicleMode("GUIDED")
#    while vehicle.mode != "GUIDED":
#        print("Waiting for vehicle to enter GUIDED mode...")
#        time.sleep(1)
	
	# Arm vehicle once GUIDED mode is confirmed
	vehicle.armed = True
#    while vehicle.armed == False:
	print("Waiting for vehicle to become armed...")
	time.sleep(5)

	vehicle.simple_takeoff(aTargetAltitude)

	# Print current altitude and stop once we are within 95% of the target altitude
	while True:
		print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= (aTargetAltitude * 0.95):
			break
		time.sleep(1)

	print("Target altitude reached.")
	return None

#converts position of aruco marker into an angle from camera
def marker_position_to_angle(x, y, z):
	
	angle_x = math.atan2(x,z)
	angle_y = math.atan2(y,z)
	
	return (angle_x, angle_y)

#Converts camera orientation to drone frame orientation
def camera_to_uav(x_cam, y_cam):
	x_uav = -x_cam
	y_uav = -y_cam
	return(x_uav, y_uav)

#converts drone frame orientation to a north east orientation
def uav_to_ne(x_uav, y_uav, yaw_rad):
	c = math.cos(yaw_rad)
	s = math.sin(yaw_rad)
	
	north = x_uav*c - y_uav*s
	east = x_uav*s + y_uav*c 
	return(north, east)

#gets current location of drone
def get_location_metres(original_location, dNorth, dEast):

	earth_radius=6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return(newlat, newlon)

#Checks angle of marker from drone frame while descending
def check_angle_descend(angle_x, angle_y, angle_desc):
	return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

#finds distance from camera to aruco marker
def find_marker_dist(corners):

	#convert camera matrix and distortion into numpy arrays
	CAMERAMATRIX = np.asarray(CAMERAMATRIX)
	CAMERADISTORTION = np.asarray(CAMERADISTORTION)

	#do maths to find marker pose from camera
	ret = aruco.estimatePoseSingleMarkers(corners, markerSize, CAMERAMATRIX, CAMERADISTORTION)

	#tvec = distance, rvec = rotation
	rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

	#grab tvec values
	x = tvec[0]
	y = tvec[1]
	z = tvec[2]

	return (x,y,z)

# Main
# Latitude and Longitude of target destinations
lat = [32.725375]
lon = [-97.129370]
alt = 4
vehicle = connectMyCopter()
print("About to take off...")

vehicle.mode = VehicleMode("GUIDED")
# Tell drone to fly specified meters into the sky (alt)
arm_and_takeoff(alt)

# Set drone airspeed (m/s)
vehicle.airspeed = 1

# Start subprocess to check for aruco marker
print('Started subprocess.\n')
process = subprocess.Popen(['python3', 'LiveArucoCodeReader.py', '-o', f'task2Test{calendar.timegm(time.gmtime())}.mp4'], stdin=None, stdout=subprocess.PIPE, text=True, universal_newlines=True)
outqueue = queue.Queue()
t = threading.Thread(target=outputReader, args=(process,outqueue))
t.start()
i = 0
brk = 0
while i < len(lat):
	# Tell drone to fly to lat and lon location at alt height
	point = LocationGlobalRelative(lat[i], lon[i], alt)
	vehicle.simple_goto(point)
	# Check the distance between current location and specified point
	while get_distance_meters(vehicle.location.global_frame, point) < 0.3:
		try:
			line = outqueue.get(block=False)
		except queue.Empty:
			continue
		if line:
			###############put drone functions here while waiting to find Aruco code
			print("Aruco Code Found...")
			brk = 1
			break
			
	i = i + 1
	if brk == 1:

		break
# Might need to check if an aruco marker is found first
while True:
	#clear the queue
	with outqueue.mutex:
		outqueue.queue.clear()
	line = None
	#try to grab from queue
	while not line:
		try:
			line = outqueue.get(block=False)
		except queue.Empty:
			continue
			
	#parse first line
	print(line)
	line = line.split(";")
	print(line)
	
	corners = line[0]
	corners = corners[1:-1]
	corners = ast.literal_eval(corners)
	for i in range(len(corners)):
		corners[i] = np.asarray(corners[i])
	corners = np.asarray(corners)
	corners = (corners,)
	CAMERAMATRIX = np.asarray(CAMERAMATRIX)
	CAMERADISTORTION = np.asarray(CAMERADISTORTION)
	ret = aruco.estimatePoseSingleMarkers(corners, 45, CAMERAMATRIX, CAMERADISTORTION)
	rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
	x = tvec[0]
	y = tvec[1]
	z = tvec[2]
	
	#convert to drone frame orientation
	x, y = camera_to_uav(x, y)
	current_location = vehicle.location.global_relative_frame

	#If high altitude, use barometer rather than visual altitude
	if current_location.alt >= 5.0:
	z = current_location.alt * 100.0 #cm

	print(f"Current altitude: {z}cm")
	
	#find angle from drone to marker
	angleX, angleY =  marker_position_to_angle(x, y, z)

	#convert drone frame orientation to a north east orientation
	north, east = uav_to_ne(x, y, vehicle.attitude.yaw)

	#get latitude and longitude of marker
	marker_lat, marker_lon = get_location_metres(current_location, north*0.01, east*0.01)

	#If angle is good, descend
	if check_angle_descend(angle_x, angle_y, angle_descend):
	print ("Descending...")
		location_marker = LocationGlobalRelative(marker_lat, marker_lon, current_location.alt-(landing_speed*0.01/1))
	else:
		location_marker = LocationGlobalRelative(marker_lat, marker_lon, current_location.alt)

	vehicle.simple_goto(location_marker)

	#Command to land if altitude is lower than set altitude
	if z <= landing_altitude:
	if vehicle.mode.name == "GUIDED":
			print (" -->>COMMANDING TO LAND<<")
			vehicle.mode = "LAND"
	
# Tell drone to land
print("Landing...")
vehicle.mode = VehicleMode("LAND")

time.sleep(10)

# Closing vehicle
print("Closing vehicle object...")
vehicle.close()

t.join()
process.terminate()

print("End of mission.")