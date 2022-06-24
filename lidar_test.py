###need to install rplidar using pip:
###pip install rplidar-roboticia
###OR for python3:
###sudo pip3 install rplidar-roboticia

import os
import sys
import time
from rplidar import RPLidar
from math import cos, sin, pi


PORT_NAME = '/dev/tty.usbserial-0001'
lidar = RPLidar(PORT_NAME)
start = time.time()
sum = 0
for i, scan in enumerate(lidar.iter_scans()):
    #print('%d: Got %d measures' %(i, len(scan)))
    
    # if sum >= 400:
    #     break
    sum = sum + len(scan)
    for(_, angle, distance) in scan:
        "hello"
        print("angle:")
        print(angle)
        print("distance")
        print(distance)
    if((i % 11) ==0):
        print('sum: %d' % sum)
        sum = 0
    if(i > 1):
        break
#print(sum)
end = time.time()

print(end - start)

# try:
#     print("starting measurements...")
#     for scan in lidar.iter_scans():
#     #     pass
#     # lcd.fill((0,0,0))
#         for(_, angle, distance) in scan:
#             print("angle:")
#             print(angle)
#             print("distance")
#             print(distance)
# except KeyboardInterrupt:
#     print("Ending...")
# lidar.stop()
lidar.stop_motor()
lidar.disconnect()