import setup_path 
import airsim

import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import random

from plot_utils import cuboid_data
from generate_valid_points import fast_generate_valid_points, generate_all_points, MapDescription
from path_plan import recursive_search

def decision(probability):
    return random.random() < probability

class lidar_plotter():

    def __init__(self, init_plot = True) -> None:
        
        if(init_plot):
            # Create plot to put data on
            self.init_plot()
            self.reset_min_max()
            self.axis_reset_timestamp = time.time()

    def init_plot(self):
        # Create plots
        self.fig = plt.figure(figsize=(7,7))
        # self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1, projection='3d')
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.points_filtered = []
        # For path planning
        path_map = MapDescription(
            -20,    20,     2.0,  # X
            -20,    20,     2.0,  # Y
            -8,     8,      2.0   # Z
        )
        self.all_points = generate_all_points(path_map)
        self.path_plan_timestamp = 0
        self.path_ready = False

    def reset_min_max(self):
        self.x_max, self.y_max, self.z_max = 0,0,0
        self.x_min, self.y_min, self.z_min = 100000000,100000000,100000000

    def update_min_max(self, client_height):
        if( (np.size(self.x_list)>0) and (np.size(self.y_list)>0) and (np.size(self.z_list)>0) ):
            # Update maxes
            self.x_max, self.y_max, self.z_max = max(self.x_max, np.max(self.x_list)), max(self.y_max, np.max(self.y_list)), max(self.z_max, np.max(self.z_list), client_height)
            # Update mins
            self.x_min, self.y_min, self.z_min = min(self.x_min, np.min(self.x_list)), min(self.y_min, np.min(self.y_list)), min(self.z_min, np.min(self.z_list))

    def filter_points(self, points: np.ndarray, z_filter: float):
        # Have something to update with
        if(points.ndim==2):
            # Sample points, correct NED z and orientation
            points_filtered = points
            points_filtered[:,0] = points_filtered[:,0]
            points_filtered[:,1] = -points_filtered[:,1]
            points_filtered[:,2] = -points_filtered[:,2]
            # Now create lists of points we want to keep
            add_index_list_1 = points_filtered[:, 2] < z_filter
            add_index_list_1 = [add_index_list_1[i] and decision(0.6) for i in range(len(add_index_list_1))] # keep 20% of the original points
            add_index_list_2 = points_filtered[:, 2] >= z_filter
            add_index_list = add_index_list_1 + add_index_list_2
            points_filtered = points_filtered[ add_index_list ]
            # Update plotted values
            self.points_filtered = points_filtered
            self.x_list,self.y_list,self.z_list = points_filtered[:,0], points_filtered[:,1], points_filtered[:,2]

    def draw_detection_boxes(self, detect_objects):

        if (len(detect_objects)==0):
                self.ax.text2D(0.45, 0.95, f"No object detected...", transform=self.ax.transAxes)        
        else:
            for detect_object in detect_objects:
                x_min, x_max = detect_object.box3D.min.x_val, detect_object.box3D.max.x_val
                y_min, y_max = -detect_object.box3D.min.y_val, -detect_object.box3D.max.y_val
                z_min, z_max = -detect_object.box3D.min.z_val, -detect_object.box3D.max.z_val
                center = [(x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2]
                length = abs(x_max-x_min)*1.2
                width = abs(y_max-y_min)*1.2
                height = abs(z_max-z_min)*1.2
                
                object_distance = detect_object.relative_pose.position.get_length()
                self.ax.text2D(0.45, 0.95, f"{detect_object.name}: {object_distance:.2f}m", transform=self.ax.transAxes)

                X, Y, Z = cuboid_data(center, (length, width, height))
                self.ax.plot_wireframe(X, Y, Z, color='b', rstride=1, cstride=1, alpha=1, edgecolor='red')

    def path_plan(self, z_offset, goal_coordinate):
        obstacles = self.points_filtered
        # Generate the points on the fly
        valid_points = fast_generate_valid_points(obstacles,self.all_points,distance_threshold=5.0)

        if not (valid_points.size==0):

            # Select the point closest to expected start location
            expected_start_location = np.array([0,0,z_offset])
            start_distances = np.linalg.norm(valid_points - expected_start_location, axis=1)
            closest_start_index = np.argmin(start_distances)

            # Select the point closest to expected stop location
            expected_stop_location = np.array([goal_coordinate[0],goal_coordinate[1],goal_coordinate[2]])
            stop_distances = np.linalg.norm(valid_points - expected_stop_location, axis=1)
            closest_stop_index = np.argmin(stop_distances)

            # Now assign start/stop
            self.start_position = valid_points[closest_start_index]
            self.stop_position = valid_points[closest_stop_index]

            # Get the neighbors
            path = recursive_search(closest_start_index,closest_stop_index,valid_points,search_radius=2.0)
            self.path_list = valid_points[path]
            self.path_ready = True

    def draw_path_plan(self):
        self.ax.scatter( self.start_position[0], self.start_position[1], -self.start_position[2],linewidths=5,c='blue')
        self.ax.scatter( self.path_list[:,0], self.path_list[:,1], -self.path_list[:,2], linewidths=2,c='red')
        self.ax.scatter( self.stop_position[0], self.stop_position[1], -self.stop_position[2], linewidths=5,c='green')

    def update_plot(self,points: np.ndarray, client:airsim.MultirotorClient, detect_objects: list, pause_time = 0.1, lidar_offset=0.15, do_pause = True):
        
        # Update height
        client_height = -client.simGetVehiclePose().position.z_val
        self.z_offset = -( client_height + lidar_offset ) 
        # Filter points
        self.filter_points(points,self.z_offset)

        # Want to reset axis limits
        if( time.time()-self.axis_reset_timestamp > 5):
            self.axis_reset_timestamp = time.time()
            self.reset_min_max()
        
        # Update min max every loop
        self.update_min_max(client_height)

        # Before drawing data, clear old data
        self.ax.clear()

        # Do path planning and draw the path
        if( (time.time()-self.path_plan_timestamp) > 1.0):
            self.path_plan_timestamp = time.time()
            self.path_plan(self.z_offset,(0,15,-6))
        if (self.path_ready):
            self.draw_path_plan()

        # Draw detection boxes
        self.draw_detection_boxes(detect_objects)

        # plot the data
        self.ax.scatter( self.x_list, self.y_list, self.z_list,
                linewidths=0.5,c=self.z_list)
        self.ax.quiver(0, 0, client_height, 3, 0, 0,linewidth=8,color='red')

        # Print the client height inside the figure
        self.ax.text2D(0.00, 0.95, f"Client Height: {client_height:.2f}m", transform=self.ax.transAxes)

        # Update limits
        self.ax.axes.set_xlim3d(left=-self.point_value_cap, right=self.point_value_cap) 
        self.ax.axes.set_ylim3d(bottom=-self.point_value_cap, top=self.point_value_cap) 
        self.ax.axes.set_zlim3d(bottom=(self.z_min-1.0), top=(self.z_max+1.0) ) 
        # Set the labels
        self.ax.axes.set_xlabel("x")
        self.ax.axes.set_ylabel("y")
        self.ax.axes.set_zlabel("z")

        # Update plot and pause
        plt.draw() 
        if(do_pause):
            plt.pause(pause_time)


    def parse_lidarData(self, data, point_value_cap=15, ground_low=-0.5, ground_high=-0.15):
        self.point_value_cap = point_value_cap
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
        # block points beyond certain range for x,y 
        points = np.array([point for point in points if ( ( abs(point[0]) <= point_value_cap) and ( abs(point[1]) <= point_value_cap))])
        # # block points that are basically just the ground
        # points = np.array([point for point in points if not ( (ground_low+z_offset <= -point[2]) and (-point[2] <= ground_high+z_offset) )  ])
        
        return points