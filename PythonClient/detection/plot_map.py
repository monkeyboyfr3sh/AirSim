import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import heapq

from generate_valid_points import generate_valid_points, load_lidar_obstacles, load_valid_points

def plot_map():
    
    # Load obstacles and valid points in the graph
    obstacles = load_lidar_obstacles()
    valid_points = load_valid_points()
    
    # Generate a random index in the range [0, len(points)-1]
    random_start_index = np.random.randint(len(valid_points))
    random_stop_index = np.random.randint(len(valid_points))
    
    # Select the random point using the random index
    start_position = valid_points[random_start_index]
    stop_position = valid_points[random_stop_index]
    

    # Create a figure
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(1,1,1, projection='3d')

    # Update plotted values
    obstacles_x_list,obstacles_y_list,obstacles_z_list = obstacles[:,0], obstacles[:,1], -obstacles[:,2]
    valid_x_list,valid_y_list,valid_z_list = valid_points[:,0], valid_points[:,1], -valid_points[:,2]

    # plot the obstacles 
    ax.scatter( obstacles_x_list, obstacles_y_list, obstacles_z_list,
            linewidths=5,c='red')
    ax.scatter( valid_x_list, valid_y_list, valid_z_list,
            linewidths=0.5,c=valid_z_list)
    ax.scatter( start_position[0], start_position[1], -start_position[2],
            linewidths=5,c='blue')
    ax.scatter( stop_position[0], stop_position[1], -stop_position[2],
            linewidths=5,c='green')

    # Update plot and pause
    plt.draw()
    plt.show()
    
if __name__ == "__main__":
    plot_map()