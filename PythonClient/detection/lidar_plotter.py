import setup_path 
import airsim

import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from plot_utils import cuboid_data

AXIS_HARD_LIMIT = 15

def parse_lidarData(data, z_offset, point_value_cap=15, ground_low=-0.5, ground_high=-0.15):

    # reshape array of floats to array of [X,Y,Z]
    points = np.array(data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    # block points beyond certain range for x,y 
    points = np.array([point for point in points if ( ( abs(point[0]) <= point_value_cap) and ( abs(point[1]) <= point_value_cap))])
    # # block points that are basically just the ground
    # points = np.array([point for point in points if not ( (ground_low+z_offset <= -point[2]) and (-point[2] <= ground_high+z_offset) )  ])
    
    return points

class lidar_plotter():

    def __init__(self) -> None:

        # Create plot to put data on
        self.init_plot()
        self.reset_min_max()
        self.axis_reset_timestamp = time.time()

    def init_plot(self):
        # Create plots
        # self.fig = plt.figure(figsize=(10,6))
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1, projection='3d')
        self.x_list = []
        self.y_list = []
        self.z_list = []

    def reset_min_max(self):
        self.x_max, self.y_max, self.z_max = 0,0,0
        self.x_min, self.y_min, self.z_min = 100000000,100000000,100000000

    def update_min_max(self):
        # Update maxes
        self.x_max, self.y_max, self.z_max = max(self.x_max, np.max(self.x_list)), max(self.y_max, np.max(self.y_list)), max(self.z_max, np.max(self.z_list))
        # Update mins
        self.x_min, self.y_min, self.z_min = min(self.x_min, np.min(self.x_list)), min(self.y_min, np.min(self.y_list)), min(self.z_min, np.min(self.z_list))


    def update_plot(self,points: np.ndarray, client:airsim.MultirotorClient, detect_objects: list, pause_time = 0.1, lidar_offset=0.25):
        client_height = client.simGetVehiclePose().position.z_val
        
        # Have something to update with
        if(points.ndim==2):
            self.x_list = points[:,0]
            self.y_list = -points[:,1]
            self.z_list = -points[:,2]

        # Want to reset axis limits
        self.update_min_max()
        if( time.time()-self.axis_reset_timestamp > 1):
            self.axis_reset_timestamp = time.time()
            self.reset_min_max()

        # Before drawing data, clear old data
        self.ax.clear()

        # Plot each detection object
        for detect_object in detect_objects:
            x_min, x_max = detect_object.box3D.min.x_val, detect_object.box3D.max.x_val
            y_min, y_max = -detect_object.box3D.min.y_val, -detect_object.box3D.max.y_val
            z_min, z_max = -detect_object.box3D.min.z_val, -detect_object.box3D.max.z_val
            center = [(x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2]
            length = abs(x_max-x_min)*1.2
            width = abs(y_max-y_min)*1.2
            height = abs(z_max-z_min)*1.2
            
            object_distance = detect_object.relative_pose.position.get_length()
            self.ax.text2D(0.5, 0.95, f"Distance: {object_distance:.2f}", transform=self.ax.transAxes)

            X, Y, Z = cuboid_data(center, (length, width, height))
            self.ax.plot_wireframe(X, Y, Z, color='b', rstride=1, cstride=1, alpha=1, edgecolor='red')

        # plot the data
        self.ax.scatter( self.x_list, self.y_list, self.z_list,
                linewidths=0.5,c=self.z_list)
        self.ax.quiver(0, 0, client_height, 3, 0, 0,linewidth=5,color='red')
        
        # Print the client height inside the figure
        self.ax.text2D(0.00, 0.95, f"Client Height: {-client_height:.2f}", transform=self.ax.transAxes)

        # Update limits
        self.ax.axes.set_xlim3d(left=-AXIS_HARD_LIMIT, right=AXIS_HARD_LIMIT) 
        self.ax.axes.set_ylim3d(bottom=-AXIS_HARD_LIMIT, top=AXIS_HARD_LIMIT) 
        self.ax.axes.set_zlim3d(bottom=-5, top=10) 
        # Set the labels
        self.ax.axes.set_xlabel("x")
        self.ax.axes.set_ylabel("y")
        self.ax.axes.set_zlabel("z")

        # Update plot and pause
        plt.draw() 
        plt.pause(pause_time)
        