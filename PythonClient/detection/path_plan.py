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

    if not (distances.size==0):
        # Find the index of the neighbor with the smallest distance
        best_neighbor_index = np.argmin(distances)
        # Return the index of the best neighbor in the valid_points array
        return np.where(np.all(valid_points == neighbors[best_neighbor_index], axis=1))[0][0]
    return None

def recursive_search(start_index, goal_index, valid_points, search_radius=10.0, max_path_length = 1000):
    # Initialize the path with the starting point
    path = [start_index]
    path_length = 0

    # Loop until the goal is reached
    while path[-1] != goal_index:
        # Get the neighbors of the current point
        neighbors = get_neighbors(path[-1], valid_points, search_radius=search_radius)

        # Choose the best neighbor
        best_neighbor_index = get_best_neighbor(goal_index, neighbors, valid_points)

        if (best_neighbor_index==None):
            return None

        # Add the best neighbor to the path
        path.append(best_neighbor_index)

        path_length+=1
        if(path_length>=max_path_length):
            return None

    # TODO: Maybe implement a pruning step?

    return path

def do_path_planning():
    PLOT_X_MIN = -20
    PLOT_X_MAX = 20
    PLOT_X_STEP = 2
    PLOT_Y_MIN = -20
    PLOT_Y_MAX = 20
    PLOT_Y_STEP = 2
    PLOT_Z_MIN = -7
    PLOT_Z_MAX = 2
    PLOT_Z_STEP = 2

    map = MapDescription(
        PLOT_X_MIN, PLOT_X_MAX, PLOT_X_STEP,
        PLOT_Y_MIN, PLOT_Y_MAX, PLOT_Y_STEP,
        PLOT_Z_MIN, PLOT_Z_MAX, PLOT_Z_STEP
    )

    # Load obstacles and valid points in the graph
    obstacles = np.genfromtxt('lidar_plot.csv', delimiter=',')
    
    # Generate the points on the fly
    valid_points = fast_generate_valid_points(obstacles,map,distance_threshold=5.0)

    # Generate a random index in the range [0, len(points)-1]
    random_start_index = np.random.randint(len(valid_points))
    random_stop_index = np.random.randint(len(valid_points))
    
    # Select the random point using the random index
    start_position = valid_points[random_start_index]
    stop_position = valid_points[random_stop_index]

    # Get the neighbors
    path = recursive_search(random_start_index,random_stop_index,valid_points,search_radius=2.0)
    path_list = valid_points[path]

    # Create a figure
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(1,1,1, projection='3d')

    # plot the obstacles 
    # ax.scatter( valid_points[:,0], valid_points[:,1], valid_points[:,2],
    #         linewidths=1,c='yellow')
    ax.scatter( obstacles[:,0], obstacles[:,1], -obstacles[:,2],
            linewidths=1,c='red')
    ax.scatter( start_position[0], start_position[1], -start_position[2],
            linewidths=5,c='blue')
    ax.scatter( path_list[:,0], path_list[:,1], -path_list[:,2],
            linewidths=2,c='purple')
    ax.scatter( stop_position[0], stop_position[1], -stop_position[2],
            linewidths=5,c='green')

    # Update plot and pause
    plt.draw()
    plt.show()

if __name__ == "__main__":
    do_path_planning()