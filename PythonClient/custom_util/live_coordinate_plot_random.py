import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time

altitude = []
latitude = []
longitude = []

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

while True:
    # Generate a new data point and append it to the lists
    new_altitude = random.uniform(0, 100)
    new_latitude = random.uniform(-90, 90)
    new_longitude = random.uniform(-180, 180)
    altitude.append(new_altitude)
    latitude.append(new_latitude)
    longitude.append(new_longitude)

    # Plot the new data point
    ax.scatter(longitude, latitude, altitude)

    # Set the axis labels
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude')

    plt.draw() # Redraw the plot
    plt.pause(0.01) # Pause for a short time

    # Clear the plot to plot new data points
    ax.clear()

plt.show()
