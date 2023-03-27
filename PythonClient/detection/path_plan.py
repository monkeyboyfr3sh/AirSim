import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.distance import cdist

from generate_valid_points import load_lidar_obstacles, load_valid_points


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

    # Return the best neighbor
    return neighbors[best_neighbor_index]

def do_path_planning():
    
    # Load obstacles and valid points in the graph
    obstacles = load_lidar_obstacles()
    valid_points = load_valid_points()
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
    neighbor_points = get_neighbors(random_start_index, valid_points)
    best_neighbor = get_best_neighbor(random_stop_index, neighbor_points, valid_points)
    neighbor_x_list,neighbor_y_list,neighbor_z_list = neighbor_points[:,0], neighbor_points[:,1], -neighbor_points[:,2]

    # Create a figure
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(1,1,1, projection='3d')


    # # plot the obstacles 
    ax.scatter( obstacles_x_list, obstacles_y_list, obstacles_z_list,
            linewidths=5,c='red')
    # ax.scatter( valid_x_list, valid_y_list, valid_z_list,
    #         linewidths=0.5,c=valid_z_list)
    
    ax.scatter( start_position[0], start_position[1], -start_position[2],
            linewidths=5,c='blue')
    ax.scatter( neighbor_x_list, neighbor_y_list, neighbor_z_list,
            linewidths=2,c='lime')
    ax.scatter( best_neighbor[0], best_neighbor[1], best_neighbor[2],
            linewidths=5,c='purple')
    
    ax.scatter( stop_position[0], stop_position[1], -stop_position[2],
            linewidths=5,c='green')

    # Update plot and pause
    plt.draw()
    plt.show()

if __name__ == "__main__":
    do_path_planning()