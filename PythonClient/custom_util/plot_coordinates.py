import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

altitude = []
latitude = []
longitude = []

with open('log.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader) # skip header row
    for row in reader:
        altitude.append(float(row[0]))
        latitude.append(float(row[1]))
        longitude.append(float(row[2]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(longitude, latitude, altitude)

ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_zlabel('Altitude')

plt.show()
