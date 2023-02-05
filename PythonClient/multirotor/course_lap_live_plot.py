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
from skspatial.objects import Sphere
import random
import socket
import struct
import time

import math

import custom_util.airsim_data_utils as airsim_utils
import  custom_util.gps_utils as gps_utils

# Where to store the data
timestamp_list = []
x_coord = []
y_coord = []
z_coord = []
v_x = []
v_y = []
v_z = []
speed_list = []

# Create plots
fig = plt.figure(figsize=(6,10))
time_ax = fig.add_subplot(2,1,1)
spatial_ax = fig.add_subplot(2,1,2, projection='3d')

if __name__=="__main__":

    SCRIPT_TIME_SECONDS = 30
    SPHERE_RADIUS = 2

    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Init the sim utilites
    gps_converter_utility = gps_utils.GPS_utils()
    start_timestamp = timestamp = airsim_utils.get_gps_timestamp(client)

    # Now run script for set period
    script_start = time.time()
    client.simPause(False)
    while( (time.time()-script_start) < SCRIPT_TIME_SECONDS ):

        # Get velocity and coordinates
        # velocity = airsim_utils.get_gps_velocity(client)
        velocity = airsim_utils.get_multirotor_linear_velocity(client)
        velocity_magnitude = velocity.get_length()
        timestamp = (airsim_utils.get_gps_timestamp(client)-start_timestamp) / 1000000000
        position = airsim_utils.get_multirotor_position(client)

        # Add the new data
        x_coord.append(position.x_val)
        y_coord.append(position.y_val)
        z_coord.append(-position.z_val)
        
        v_x.append(velocity.x_val)
        v_y.append(velocity.y_val)
        v_z.append(velocity.z_val)
        
        timestamp_list.append(timestamp)
        speed_list.append(velocity_magnitude)

        # Plot the new data point
        time_ax.scatter(timestamp_list, speed_list, c = speed_list , cmap = "magma")
        spatial_ax.quiver(position.x_val, position.y_val, -position.z_val, velocity.x_val, velocity.y_val, -velocity.z_val)
        draw_sphere_radius = max(max(speed_list),SPHERE_RADIUS)
        sphere = Sphere([position.x_val, position.y_val, -position.z_val], draw_sphere_radius)
        sphere.plot_3d(spatial_ax, alpha=0.2)
        spatial_ax.scatter(x_coord,y_coord,z_coord,c = speed_list , cmap = "magma")

        # Set the axis labels
        time_ax.set_title('Speed vs Time')
        time_ax.set_xlabel('Time')
        time_ax.set_ylabel('Speed')
        
        spatial_ax.set_title('Position (heat map by speed)')
        spatial_ax.set_xlabel('x-coord')
        spatial_ax.set_ylabel('y-cood')
        spatial_ax.set_zlabel('z-coord')

        plt.draw() # Redraw the plot
        plt.pause(0.1) # Pause for a short time

        # Clear the plot to plot new data points
        time_ax.clear()
        spatial_ax.clear()
        spatial_ax.clear()
    
    client.simPause(True)

    while(1):
        # Plot the new data point
        time_ax.scatter(timestamp_list, speed_list, c = speed_list , cmap = "magma")
        spatial_ax.quiver(position.x_val, position.y_val, -position.z_val, velocity.x_val, velocity.y_val, velocity.z_val)
        draw_sphere_radius = max(max(speed_list),SPHERE_RADIUS)
        sphere = Sphere([position.x_val, position.y_val, -position.z_val], draw_sphere_radius)
        sphere.plot_3d(spatial_ax, alpha=0.2)
        spatial_ax.scatter(x_coord,y_coord,z_coord,c = speed_list , cmap = "magma")

        # Set the axis labels
        time_ax.set_title('Velocity vs Time')
        time_ax.set_xlabel('Time')
        time_ax.set_ylabel('Speed')

        spatial_ax.set_title('Position (heat map by velocity)')
        spatial_ax.set_xlabel('x-coord')
        spatial_ax.set_ylabel('y-cood')
        spatial_ax.set_zlabel('z-coord')

        plt.draw() # Redraw the plot
        input("Now waiting to view data... Press enter to kill script.")
        break
