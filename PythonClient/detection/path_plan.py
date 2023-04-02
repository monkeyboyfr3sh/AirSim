import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.distance import cdist

from generate_valid_points import fast_generate_valid_points, MapDescription

def get_neighbors(position_index, valid_points, search_radius=10.0):
    # Compute the Euclidean distances between all points in the array and the target point
    distances = np.linalg.norm(valid_points - valid_points[position_index], axis=1)

    # Find the indices of all points within the search radius
    neighbor_indices = np.where(distances <= search_radius)[0]

    # Remove the position_index from the neighbor_indices
    neighbor_indices = neighbor_indices[neighbor_indices != position_index]

    neighbor_points = valid_points[neighbor_indices]
    return neighbor_points

def get_best_neighbor(goal_index, neighbors, valid_points):
    # Compute the Euclidean distances between all points in the array and the target point
    distances = np.linalg.norm(neighbors - valid_points[goal_index], axis=1)

    # Find the index of the neighbor with the smallest distance
    best_neighbor_index = np.argmin(distances)

    # Return the index of the best neighbor in the valid_points array
    return np.where(np.all(valid_points == neighbors[best_neighbor_index], axis=1))[0][0]

def recursive_search(start_index, goal_index, valid_points, search_radius=10.0):
    # Initialize the path with the starting point
    path = [start_index]

    # Loop until the goal is reached
    while path[-1] != goal_index:
        # Get the neighbors of the current point
        neighbors = get_neighbors(path[-1], valid_points, search_radius=search_radius)

        # Choose the best neighbor
        best_neighbor_index = get_best_neighbor(goal_index, neighbors, valid_points)

        # Add the best neighbor to the path
        path.append(best_neighbor_index)

    return path

def do_path_planning():
    PLOT_X_MIN = -90
    PLOT_X_MAX = 90
    PLOT_Y_MIN = -90
    PLOT_Y_MAX = 90
    PLOT_Z_MIN = -30
    PLOT_Z_MAX = 5
    DEFAULT_VALID_FILENAME = 'valid_points_plot.csv'
    DEFAULT_OBSTACLE_FILENAME = 'lidar_plot.csv'

    map = MapDescription(
        PLOT_X_MIN, PLOT_X_MAX,
        PLOT_Y_MIN, PLOT_Y_MAX,
        PLOT_Z_MIN, PLOT_Z_MAX
    )

    # Load obstacles and valid points in the graph
    obstacles = np.genfromtxt('lidar_plot.csv', delimiter=',')
    
    # Generate the points on the fly
    valid_points = fast_generate_valid_points(obstacles,map)
    # # Load previously generated points 
    # valid_points = np.genfromtxt("fast_algo_valid_points.csv", delimiter=',')

    # Update plotted values
    obstacles_x_list,obstacles_y_list,obstacles_z_list = obstacles[:,0], obstacles[:,1], -obstacles[:,2]
    valid_x_list,valid_y_list,valid_z_list = valid_points[:,0], valid_points[:,1], -valid_points[:,2]

    # Generate a random index in the range [0, len(points)-1]
    random_start_index = np.random.randint(len(valid_points))
    random_stop_index = np.random.randint(len(valid_points))
    
    # Select the random point using the random index
    start_position = valid_points[random_start_index]
    stop_position = valid_points[random_stop_index]

    # Get the neighbors
    path = recursive_search(random_start_index,random_stop_index,valid_points)
    path_list = valid_points[path]
    path_list_x, path_list_y, path_list_z = path_list[:,0], path_list[:,1], path_list[:,2],

    # Create a figure
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(1,1,1, projection='3d')

    # # plot the obstacles 
    ax.scatter( obstacles_x_list, obstacles_y_list, obstacles_z_list,
            linewidths=1,c='red')
    # ax.scatter( valid_x_list, valid_y_list, valid_z_list,
    #         linewidths=0.5,c=valid_z_list)
    
    ax.scatter( start_position[0], start_position[1], -start_position[2],
            linewidths=5,c='blue')
    # ax.scatter( neighbor_x_list, neighbor_y_list, neighbor_z_list,
    #         linewidths=2,c='lime')
    ax.scatter( path_list_x, path_list_y, -path_list_z,
            linewidths=2,c='purple')
    
    ax.scatter( stop_position[0], stop_position[1], -stop_position[2],
            linewidths=5,c='green')

    # Update plot and pause
    plt.draw()
    plt.show()

if __name__ == "__main__":
    do_path_planning()