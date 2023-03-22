import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# load data from the CSV file
data = np.genfromtxt('lidar_plot.csv', delimiter=',')

fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(1,1,1, projection='3d')

# Update plotted values
x_list,y_list,z_list = data[:,0], data[:,1], -data[:,2]

# plot the data
ax.scatter( x_list, y_list, z_list,
        linewidths=0.5,c=z_list)
# ax.quiver(0, 0, client_height, 3, 0, 0,linewidth=8,color='red')

# Print the client height inside the figure
# ax.text2D(0.00, 0.95, f"Client Height: {client_height:.2f}m", transform=ax.transAxes)

# # Update limits
# ax.axes.set_xlim3d(left=-point_value_cap, right=point_value_cap) 
# ax.axes.set_ylim3d(bottom=-point_value_cap, top=point_value_cap) 
# ax.axes.set_zlim3d(bottom=(z_min-1.0), top=(z_max+1.0) ) 
# Set the labels
ax.axes.set_xlabel("x")
ax.axes.set_ylabel("y")
ax.axes.set_zlabel("z")

# Update plot and pause
plt.draw()
while True:
    plt.pause(1)