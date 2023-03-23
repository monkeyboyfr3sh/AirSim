import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import heapq

from generate_valid_points import generate_valid_points

def heuristic(current, goal):
    return np.sqrt(np.sum((current - goal) ** 2))

def astar(start, goal, data):
    # Initialize the start node
    start_node = (heuristic(start, goal), start, 0, None)
    # Initialize the priority queue
    queue = [start_node]
    # Initialize the visited set
    visited = set()

    # Define the modified cost function
    def cost(current, next):
        return np.sqrt(np.sum((next - current) ** 2)) + abs(current[2] - next[2])

    # While there are nodes to visit
    while queue:
        # Get the node with the lowest f-value
        _, current, g, parent = heapq.heappop(queue)

        # Check if the current node is the goal
        if np.array_equal(current, goal):
            path = []
            while parent:
                path.append(parent)
                parent = parent[3]
            return path[::-1]

        # Add the current node to the visited set
        visited.add(tuple(current))

        # Generate the successors of the current node
        for i in range(data.shape[0]):
            successor = data[i, :]
            if tuple(successor) not in visited:
                new_g = g + cost(current, successor)
                f = new_g + heuristic(successor, goal)
                new_node = (f, successor, new_g, (current, successor))
                heapq.heappush(queue, new_node)

    # No path found
    return None

def plot_map():
    
    # Load obstacles and valid points in the graph
    obstacles = np.genfromtxt('lidar_plot.csv', delimiter=',')
    valid_points = np.genfromtxt('valid_points_plot.csv', delimiter=',')
    
    # Generate a random index in the range [0, len(points)-1]
    random_start_index = np.random.randint(len(valid_points))
    random_stop_index = np.random.randint(len(valid_points))
    
    # Select the random point using the random index
    start_position = valid_points[random_start_index]
    stop_position = valid_points[random_stop_index]
    
    path = astar(start_position,stop_position,valid_points)

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