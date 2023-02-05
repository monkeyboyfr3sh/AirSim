import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import socket
import struct
import time

altitude = []
latitude = []
longitude = []

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(('127.0.0.1', 20001)) # Bind to all available interfaces on port 20001

while True:
    # Receive new data over the UDP socket
    bytes_received, _ = udp_socket.recvfrom(1024) # 1024 is the buffer size
    data = struct.unpack("3f",bytes_received)

    # Add the new data
    altitude.append(data[0])
    latitude.append(data[1])
    longitude.append(data[2])

    # Plot the new data point
    ax.scatter(longitude, latitude, altitude)

    # Set the axis labels
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude')

    plt.draw() # Redraw the plot
    plt.pause(0.1) # Pause for a short time

    # Clear the plot to plot new data points
    ax.clear()

plt.show()
