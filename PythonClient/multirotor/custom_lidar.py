import setup_path 
import airsim

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

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

def LidarRun():

    # Create plots
    fig = plt.figure(figsize=(10,6))
    ax = fig.add_subplot(1,1,1, projection='3d')

    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # client.enableApiControl(True)
    
    # Keep track of max values
    x_max, y_max, z_max = 0,0,0
    x_min, y_min, z_min = 100000000,100000000,100000000

    axis_reset_timestamp = time.time()

    while True:
        # Get Lidar data
        lidarData = client.getLidarData()
        client_height = client.simGetVehiclePose().position.z_val
        points = parse_lidarData(lidarData,client_height)
        
        # Have something to update with
        if(points.ndim==2):
            x_list = points[:,0]
            y_list = -points[:,1]
            z_list = -points[:,2]

        # Want to reset axis limits
        if( time.time()-axis_reset_timestamp > 1):
            axis_reset_timestamp = time.time()
            x_max, y_max, z_max = 0,0,0
            x_min, y_min, z_min = 100000000,100000000,100000000

        # Update maxes
        x_max, y_max, z_max = max(x_max, np.max(x_list)), max(y_max, np.max(y_list)), max(z_max, np.max(z_list))
        # Update mins
        x_min, y_min, z_min = min(x_min, np.min(x_list)), min(y_min, np.min(y_list)), min(z_min, np.min(z_list))

        # Before drawing data, clear old data
        ax.clear()

        # plot the data
        ax.scatter( x_list, y_list, z_list,
                linewidths=0.5,c=z_list)
        ax.quiver(0, 0, client_height, 3, 0, 0,linewidth=20,color='red')
        
        # Update limits
        ax.axes.set_xlim3d(left=-AXIS_HARD_LIMIT, right=AXIS_HARD_LIMIT) 
        ax.axes.set_ylim3d(bottom=-AXIS_HARD_LIMIT, top=AXIS_HARD_LIMIT) 
        ax.axes.set_zlim3d(bottom=-5, top=10) 
        # Set the labels
        ax.axes.set_xlabel("x")
        ax.axes.set_ylabel("y")
        ax.axes.set_zlabel("z")

        # Update plot and pause
        plt.draw() 
        plt.pause(0.1)

if __name__ == "__main__":
    LidarRun()