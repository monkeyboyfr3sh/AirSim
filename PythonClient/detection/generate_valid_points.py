import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

PLOT_X_MIN = -90
PLOT_X_MAX = 90
PLOT_Y_MIN = -90
PLOT_Y_MAX = 90
PLOT_Z_MIN = -30
PLOT_Z_MAX = 5

def generate_valid_points(distance_threshold : int = 10):
    obstacles = np.genfromtxt('lidar_plot.csv', delimiter=',')
    
    # Generate an array of points within the cube
    all_points = np.array([(x, y, z) 
                        for x in range(PLOT_X_MIN,PLOT_X_MAX+1,10) 
                        for y in range(PLOT_Y_MIN,PLOT_Y_MAX+1,10) 
                        for z in range(PLOT_Z_MIN,PLOT_Z_MAX+1,5)])

    # Loop over all_points and remove any that are close to obstacle points
    for i in range(len(all_points)):
        for j in range(len(obstacles)):
            distance = np.linalg.norm(all_points[i] - obstacles[j])
            if distance < distance_threshold:
                all_points[i] = np.zeros_like(all_points[i])
                
    # Remove any zero values from all_points
    valid_points = all_points[~np.all(all_points == 0, axis=1)]

    # Save the valid points
    np.savetxt('valid_points_plot.csv', valid_points, delimiter=',')

if __name__ == "__main__":
    generate_valid_points()