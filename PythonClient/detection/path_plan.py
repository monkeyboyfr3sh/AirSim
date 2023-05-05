import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.distance import cdist

from generate_valid_points import fast_generate_valid_points, generate_all_points, MapDescription

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

def recursive_search(start_index, goal_index, valid_points, search_radius=10.0, max_path_length = 1000, final_distance_threshold = 0):
    # Initialize the path with the starting point
    path = [start_index]
    path_length = 0

    # Loop until the goal is reached
    while path[-1] != goal_index:

        # Check distance of last checkpoint from goal, break if less than threshold
        distance = np.linalg.norm(valid_points[path[-1]] - valid_points[goal_index])
        if (distance < final_distance_threshold) and (final_distance_threshold != 0):
            break

        # Get the neighbors of the current point
        neighbors = get_neighbors(path[-1], valid_points, search_radius=search_radius)

        # Choose the best neighbor
        best_neighbor_index = get_best_neighbor(goal_index, neighbors, valid_points)

        if (best_neighbor_index==None):
            return []

        # Add the best neighbor to the path
        path.append(best_neighbor_index)

        path_length+=1
        if(path_length>=max_path_length):
            return []

    # TODO: Maybe implement a pruning step?

    return path

def do_path_planning():

    z_max = 10
    z_offset = -np.random.randint(z_max)
    plot_valid_points = False
    plot_obstacles = False

    # Load obstacles and valid points in the graph
    obstacles = np.genfromtxt('lidar_plot.csv', delimiter=',')
    
    # Generate the points on the fly
    # For path planning
    path_map = MapDescription(
        -100,    100,     2.0,  # X
        -100,    100,     2.0,  # Y
        -4+z_offset,      4+z_offset,       2.0   # Z
    )
    all_points = generate_all_points(path_map)
    # Generate the points on the fly
    valid_points = fast_generate_valid_points(obstacles,all_points,distance_threshold=5.0)

    if not (valid_points.size==0):

        # Generate a random index in the range [0, len(points)-1]
        random_stop_index = np.random.randint(len(valid_points))
        
        # Select the random point using the random index
        start_position = np.array([0,0,z_offset])
        stop_position = valid_points[random_stop_index]


        # Select the point closest to expected start location
        expected_start_location = np.array([0,0,z_offset])
        start_distances = np.linalg.norm(valid_points - expected_start_location, axis=1)
        closest_start_index = np.argmin(start_distances)

        # Select the point closest to expected stop location
        expected_stop_location = np.array([stop_position[0],stop_position[1],stop_position[2]])
        stop_distances = np.linalg.norm(valid_points - expected_stop_location, axis=1)
        closest_stop_index = np.argmin(stop_distances)

        # Get the path
        path = recursive_search(closest_start_index,closest_stop_index,valid_points,search_radius=4.0,final_distance_threshold=5.0)
        if(len(path)>0):

            # Get the path
            path_list = valid_points[path]

            # Create a figure
            fig = plt.figure(figsize=(7,7))
            ax = fig.add_subplot(1,1,1, projection='3d')

            # Plot the start, stop, and path
            ax.scatter( start_position[0], start_position[1], -start_position[2],linewidths=5,c='blue',label='Start Position')
            ax.scatter( path_list[:,0], path_list[:,1], -path_list[:,2], linewidths=2,c='red',label='Planned Path')
            ax.scatter( stop_position[0], stop_position[1], -stop_position[2], linewidths=5,c='green', label='Stop Position')

            # plot the valid points
            if plot_valid_points:
                ax.scatter( valid_points[:,0], valid_points[:,1], valid_points[:,2],
                        linewidths=1,c='purple')

            # plot the obstacles 
            if plot_obstacles:
                ax.scatter( obstacles[:,0], obstacles[:,1], -obstacles[:,2],
                        linewidths=1,c=-obstacles[:,2],cmap='magma')

            # Set the labels
            ax.axes.set_xlabel("x")
            ax.axes.set_ylabel("y")
            ax.axes.set_zlabel("z")
            ax.axes.set_title("Path Planning Demo")
            ax.legend()

            # Update plot and pause
            plt.draw()
            plt.show()

        else:
            print("No valid path found")
    else:
        print("No valid points found")

if __name__ == "__main__":
    do_path_planning()