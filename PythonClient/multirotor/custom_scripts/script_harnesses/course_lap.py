import get_custom_paths
import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

import time
import string

import csv   

import socket
import struct

def get_gps_coorinates(client: airsim.MultirotorClient):
    gps_data = client.getGpsData()
    geo_point = gps_data.gnss.geo_point
    coordinates = [geo_point.altitude,geo_point.latitude,geo_point.longitude]
    return coordinates

def print_coordinates(client: airsim.MultirotorClient):
    coordinates = get_gps_coorinates(client)
    print(coordinates)

def write_coordinates_to_csv(csv_filename: string, client: airsim.MultirotorClient, overwrite_file: bool):
    coordinates = get_gps_coorinates(client=client)
    
    check_file = not( os.path.isfile(csv_filename) )
    if (check_file or overwrite_file):
        with open(r'{}'.format(csv_filename), 'w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Altitude","Latitude","Longitude"])

    with open(r'{}'.format(csv_filename), 'a',newline='') as f:
        writer = csv.writer(f)
        writer.writerow(coordinates)

if __name__=="__main__":
    LOG_FILENAME = 'log.csv'

    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # client.enableApiControl(True)

    # Configure port settings
    serverAddressPort   = ("127.0.0.1", 20001)
    bufferSize          = 1024
    # Create a UDP socket at client side
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    create_file = True
    while(1):
        # airsim.wait_key('Press any key to print data')
        write_coordinates_to_csv(LOG_FILENAME,client,create_file)
        create_file = False
        coordinates = get_gps_coorinates(client)
        buf = struct.pack('%sf' % len(coordinates), *coordinates)
        # Send to server using created UDP socket
        UDPClientSocket.sendto(buf, serverAddressPort)
        time.sleep(0.1)

# state = client.getMultirotorState()
# s = pprint.pformat(state)
# print("state: %s" % s)

# imu_data = client.getImuData()
# s = pprint.pformat(imu_data)
# print("imu_data: %s" % s)

# barometer_data = client.getBarometerData()
# s = pprint.pformat(barometer_data)
# print("barometer_data: %s" % s)

# magnetometer_data = client.getMagnetometerData()
# s = pprint.pformat(magnetometer_data)
# print("magnetometer_data: %s" % s)

# gps_data = client.getGpsData()
# s = pprint.pformat(gps_data)
# print("gps_data: %s" % s)

# airsim.wait_key('Press any key to takeoff')
# print("Taking off...")
# client.armDisarm(True)
# client.takeoffAsync().join()

# state = client.getMultirotorState()
# print("state: %s" % pprint.pformat(state))

# airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
# client.moveToPositionAsync(-10, 10, -10, 5).join()

# client.hoverAsync().join()