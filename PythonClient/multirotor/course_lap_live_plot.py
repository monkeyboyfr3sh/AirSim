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

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import socket
import struct
import time

import math

import  custom_util.gps_utils as gps_utils

altitude = []
latitude = []
longitude = []


timestamp_list = []
x_coord = []
y_coord = []
z_coord = []
v_x = []
v_y = []
v_z = []
speed = []

fig = plt.figure(figsize=(5,10))

# ax = fig.add_subplot(2,1,1, projection='3d')
ax2 = fig.add_subplot(2,1,1, projection='3d')
ax3 = fig.add_subplot(2,1,2)

def get_gps_coorinates(client: airsim.MultirotorClient):
    gps_data = client.getGpsData()
    geo_point = gps_data.gnss.geo_point
    coordinates = [geo_point.altitude,geo_point.latitude,geo_point.longitude]
    return coordinates

def get_velocity(client: airsim.MultirotorClient):
    gps_data = client.getGpsData()
    velocity = gps_data.gnss.velocity
    velocity = [velocity.x_val,velocity.y_val,velocity.z_val]
    return velocity

def get_timestamp(client: airsim.MultirotorClient):
    gps_data = client.getGpsData()
    return gps_data.time_stamp

def magnitude(vector):
    return math.sqrt(sum(pow(element, 2) for element in vector))

if __name__=="__main__":

    SCRIPT_TIME_SECONDS = 30

    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Init the sim utilites
    gps_converter_utility = gps_utils.GPS_utils()
    coordinates = get_gps_coorinates(client)
    gps_converter_utility.setENUorigin(coordinates[2],coordinates[1],coordinates[0])
    start_timestamp = timestamp = get_timestamp(client)

    # Now run script for set period
    script_start = time.time()
    client.simPause(False)
    while( (time.time()-script_start) < SCRIPT_TIME_SECONDS ):

        # Get velocity and coordinates
        velocity = get_velocity(client)
        velocity_magnitude = magnitude(velocity)
        coordinates = get_gps_coorinates(client)
        timestamp = (get_timestamp(client)-start_timestamp) / 1000000000
        position = client.getMultirotorState().kinematics_estimated.position

        # Add the new data
        altitude.append(coordinates[0])
        latitude.append(coordinates[1])
        longitude.append(coordinates[2])

        x_coord.append(position.x_val)
        y_coord.append(position.y_val)
        z_coord.append(-position.z_val)
        
        v_x.append(velocity[0])
        v_y.append(velocity[1])
        v_z.append(velocity[2])
        
        timestamp_list.append(timestamp)
        speed.append(velocity_magnitude)

        # Plot the new data point
        # ax.scatter(longitude, latitude, altitude, c = speed , cmap = "magma")
        ax2.scatter(x_coord,y_coord,z_coord,c = speed , cmap = "magma")
        ax3.scatter(timestamp_list, speed, c = speed , cmap = "magma")

        # # Set the axis labels
        # ax.set_xlabel('Longitude')
        # ax.set_ylabel('Latitude')
        # ax.set_zlabel('Altitude')

        ax2.set_title('Position (heat map by velocity)')
        ax2.set_xlabel('x-coord')
        ax2.set_ylabel('y-cood')
        ax2.set_zlabel('z-coord')

        ax3.set_title('Velocity vs Time')
        ax3.set_xlabel('Time')
        ax3.set_ylabel('Speed')

        plt.draw() # Redraw the plot
        plt.pause(0.1) # Pause for a short time

        # Clear the plot to plot new data points
        # ax.clear()
        ax2.clear()
        ax3.clear()
    
    client.simPause(True)

    while(1):
        # Plot the new data point
        # ax.scatter(longitude, latitude, altitude, c = speed , cmap = "magma")
        ax2.scatter(x_coord,y_coord,z_coord,c = speed , cmap = "magma")
        ax3.scatter(timestamp_list, speed, c = speed , cmap = "magma")

        # # Set the axis labels
        # ax.set_xlabel('Longitude')
        # ax.set_ylabel('Latitude')
        # ax.set_zlabel('Altitude')

        ax2.set_title('Position (heat map by velocity)')
        ax2.set_xlabel('x-coord')
        ax2.set_ylabel('y-cood')
        ax2.set_zlabel('z-coord')

        ax3.set_title('Velocity vs Time')
        ax3.set_xlabel('Time')
        ax3.set_ylabel('Speed')
        plt.draw() # Redraw the plot
        input("Now waiting to view data... Press enter to kill script.")
        break
