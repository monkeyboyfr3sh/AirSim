import setup_path 
import airsim
from airsim import Vector3r, Quaternionr, Pose
from airsim.utils import to_quaternion
import cv2
import numpy as np 

import time
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import sys
import time
import threading

import detection_utils as dt_util
from lidar_plotter import LidarPlotter
from queue import Queue

def client_takeoff(client:airsim.MultirotorClient,z: float):
    # Take API control
    print(f"Acquiring API control...",end=' ')
    client.enableApiControl(True)
    
    # Takeoff
    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print(f"Taking off...",end=' ')
        client.takeoffAsync().join()
    else:
        print(f"Hovering...",end=' ')
        client.hoverAsync().join()

    # AirSim uses NED coordinates so negative axis is up.
    print(f"Making sure hover at: {-z} meters...",end=' ')
    client.moveToZAsync(z, 1).join()
    print(f"Takeoff complete!")

def client_disarm(client:airsim.MultirotorClient):
    client.hoverAsync().join()
    client.moveToZAsync(0, 1).join()
    client.enableApiControl(False)

def navigate_to_monument(client:airsim.MultirotorClient, z: float, hover_duration: float = 5.0):
    
    # Takeoff
    client_takeoff(client=client,z=z)
    time.sleep(1)
    
    # Schedule the flight path
    print("Flying to monument...",end=' ')
    path_1 = [  airsim.Vector3r(127,1.0,z),
                airsim.Vector3r(127,85,z) ]
    
    path_2 = [  airsim.Vector3r(127,120,z) ]
    
    path_3 = [ airsim.Vector3r(110,128,z),
             airsim.Vector3r(110,155,z) ]
    print("Starting path 1...",end=' ')
    client.moveOnPathAsync(
                            path=path_1,
                            velocity=10, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=20, adaptive_lookahead=1 ).join()
    print("Starting path 2...",end=' ')
    client.moveOnPathAsync(
                            path=path_2,
                            velocity=10, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=10, adaptive_lookahead=1 ).join()
    print("Starting path 3...",end=' ')
    client.moveOnPathAsync(
                            path=path_3,
                            velocity=10, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=10, adaptive_lookahead=1 ).join()
    print(f"Now hovering for {hover_duration}s...",end=' ')
    start_time = time.time()
    while(time.time()-start_time < hover_duration):
        client.hoverAsync().join()
    
    print(f"Now landing... ",end=' ')
    client_disarm(client=client)

    print('Navigation complete!')

def move_distance_from_monument(client:airsim.MultirotorClient, z: float, monument_name: str, distance: float , hover_duration: float):
    print(f"Moving to be {distance}m wrt monument...")

    # Takeoff
    client_takeoff(client=client,z=z)
    time.sleep(1)

    # First center the drone 
    dt_util.center_on_detection(client,monument_name)
    dt_util.move_to_distance_from_object(client,monument_name,z,distance, velocity = (2.5, 0.0, 0.0))
    # Center the drone 
    dt_util.center_on_detection(client,monument_name)

    client_disarm(client=client)
    print('Navigation complete!')

def get_close_point(client:airsim.MultirotorClient, lidar_plot: LidarPlotter, close_threshold: float):
    # Get Lidar data
    lidarData = client.getLidarData()
    points = lidar_plot.parse_lidarData(lidarData,point_value_cap=50)
    # Get the mgnitude for each point
    magnitudes = np.array([np.linalg.norm(point) for point in points])
    # Now get the points that are too close
    close_points = np.array(points[(magnitudes < close_threshold)])
    # FIXME: This is probably terrible but idk what else to do rn
    average_location = np.mean(close_points, axis=0)
    return average_location

def avoid_hover(client:airsim.MultirotorClient, z: float, close_threshold: float, hover_duration: float):
    # print("Avoid hovering...")

    # Planning to use lidar data, so create the LidarPlotter
    lidar_plot = LidarPlotter(init_plot=False)

    # Takeoff
    # client_takeoff(client=client,z=z)
    average_location = get_close_point(client=client,lidar_plot=lidar_plot,close_threshold=close_threshold)
    print(average_location)

def save_data(client:airsim.MultirotorClient):
    print("Saving lidar data to a csv...",end=' ')

    # Planning to use lidar data, so create the LidarPlotter
    lidar_plot = LidarPlotter(init_plot=False)

    # Read the lidar data
    lidarData = client.getLidarData()
    points = lidar_plot.parse_lidarData(lidarData,point_value_cap=100)
    np.savetxt('lidar_plot.csv', points, delimiter=',')
    
    print("Complete!")

def path_plan_to_target(client:airsim.MultirotorClient, target_name: str, lidar_offset=0.15):
    print("Running path planning...",end=' ')

    # Take API control
    print(f"Acquiring API control...",end=' ')
    client.enableApiControl(True)
    client.simPause(False)
    client_position = client.simGetVehiclePose().position

    # Takeoff
    print(f"Taking off...",end=' ')
    client.takeoffAsync().join()

    # AirSim uses NED coordinates so negative axis is up.
    z=client_position.z_val
    print(f"Making sure hover at: {-z} meters...",end=' ')
    client.moveToZAsync(z, 1).join()
    print(f"Takeoff complete!")
    time.sleep(1) # TODO: should just poll till 'stable' 

    # First center the drone 
    dt_util.center_on_detection(client,target_name)

    # Get orientation, and yaw
    client_orientation = client.simGetVehiclePose().orientation
    pitch, roll, yaw = airsim.utils.to_eularian_angles(client_orientation)

    # Get XYZ of target
    target_coord = dt_util.get_detect_coordinates(client, target_name)

    client.simPause(True)

    # Only work if object to dete
    if not(target_coord==None):
        print(f"detected {target_name} at {target_coord}...",end=' ')

        # Create a plotter for lidar data
        lidar_plot = LidarPlotter()

        # Get Lidar data
        lidarData = client.getLidarData()
        points = lidar_plot.parse_lidarData(lidarData,point_value_cap=100)

        # Update the plot
        lidar_plot.update_plot(points,client,[],do_pause=False,show_filtered=False)

        # Do path planning and draw the path
        path_list = lidar_plot.path_plan(lidar_plot.z_offset,target_coord,plot_path=True)

        if ( len(path_list) > 0 ):
 
            # Convert path to numpy array
            flight_path = np.asarray(path_list)

            # Save and load the flight path
            np.savetxt("flight_path.csv", flight_path, delimiter=',')
            flight_path = np.genfromtxt("flight_path.csv", delimiter=',')

            # Rotate the flight path
            rotated_flight_path = dt_util.rotate_path_from_yaw(flight_path,yaw)

            # Show to goal points in the sim enviroment
            flight_vectors = [ airsim.Vector3r(path_point[0]+client_position.x_val,path_point[1]+client_position.y_val,path_point[2]) for path_point in rotated_flight_path ]
            client.simPlotPoints(   points = flight_vectors, color_rgba=[1.0, 1.0, 0.0, 1.0], size = 1, duration = 30.0, is_persistent = False)
            client.simPause(False)

            # Wait for lidar plot to be closed
            client.simPause(True)
            plt.show()

            # Show to goal points in the sim enviroment
            client.simPlotPoints(   points = flight_vectors, color_rgba=[1.0, 1.0, 0.0, 1.0], size = 10, duration = 30.0, is_persistent = False)

            # Now unpause and follow the path
            client.simPause(False)

            client.moveOnPathAsync(
                                    path=flight_vectors,
                                    velocity=5, timeout_sec=120,
                                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                                    yaw_mode=airsim.YawMode(False,0),
                                    lookahead=1, adaptive_lookahead=1 ).join()

            # Hover at the end of a task
            z = -4
            print(f"Making sure hover at: {z} meters...",end=' ')
            client.moveToZAsync(z, 2).join()
            time.sleep(1)

            # # First center the drone 
            # dt_util.center_on_detection(client,target_name)
        else:
            print("No path found")
    else:
        print(f"{target_name} not detected...",end=' ')
        client.simPause(False)

    client.enableApiControl(False)
    print("Complete!")

def rotate_path(client:airsim.MultirotorClient):

    # Get position, orientation, and yaw
    client_position = client.simGetVehiclePose().position
    client_orientation = client.simGetVehiclePose().orientation
    pitch, roll, yaw = airsim.utils.to_eularian_angles(client_orientation)
    
    # Load the path and convert it based on a yaw
    initial_path = np.genfromtxt("flight_path.csv", delimiter=',')
    rotated_path = dt_util.rotate_path_from_yaw(initial_path,yaw)

    # Show to goal points in the sim enviroment
    client.simPlotPoints(   points = [ airsim.Vector3r(path_point[0]+client_position.x_val,path_point[1]+client_position.y_val,path_point[2]) for path_point in initial_path ],
                            color_rgba=[1.0, 0.0, 0.0, 1.0], size = 25, duration = 1.0, is_persistent = False)
    client.simPlotPoints(   points = [ airsim.Vector3r(path_point[0]+client_position.x_val,path_point[1]+client_position.y_val,path_point[2]) for path_point in rotated_path ],
                        color_rgba=[1.0, 1.0, 0.0, 1.0], size = 25, duration = 1.0, is_persistent = False)

def create_task_client(target,args=None,start_task=False) -> threading.Thread:
        # Create a client for the tasks to share
        task_client = airsim.MultirotorClient()
        task_client.confirmConnection()

        # Prepend the client into tuple
        if(args==None):
            args = (task_client, ) 
        else:   
            args = (task_client, ) + args

        # should now have: args = (task_client) or (task_client, arg1, arg2, ...)
        
        # Create thread
        task_thread = threading.Thread(target=target, args=args)
        if (start_task):
            task_thread.start()
        return task_thread, task_client

def viewer_task(z: float, png_queue: Queue):

    target_name = "Monument_01_176"
    # target_name = "Car_35"
    default_z = z
    task_thread = threading.Thread()

    while (True):
        
        # Check if the queue has a new image
        if( not (png_queue.empty()) ):

            # Get the picture from queue
            png = png_queue.get()

            # Show image and take in user input
            cv2.imshow("AirSim", png)
        
        # Always poll the key stroke        
        key = cv2.waitKey(1)
        if(key!=-1):
            print(f"Received key: {chr(key)}")

        # Always give the option to break program
        if key & 0xFF == ord('q'):
            break

        # Previous thread is done, can start a new task
        if not (task_thread.is_alive()):
            if key & 0xFF == ord('d'):
                task_thread, task_client = create_task_client(target=client_disarm,start_task=True)
            elif key & 0xFF == ord('t'):
                task_thread, task_client = create_task_client(target=client_takeoff,args=(default_z,),start_task=True)
            elif key & 0xFF == ord('n'):
                task_thread, task_client = create_task_client(target=navigate_to_monument,args=(default_z,),start_task=True)
            elif key & 0xFF == ord('m'):
                task_thread, task_client = create_task_client(target=move_distance_from_monument,args=(default_z,target_name, 10.0,5.0),start_task=True)
            elif key & 0xFF == ord('h'):
                task_thread, task_client = create_task_client(target=avoid_hover,args=(default_z, 5.0, 5.0),start_task=True)
            elif key & 0xFF == ord('s'):
                task_thread, task_client = create_task_client(target=save_data,start_task=True)
            elif key & 0xFF == ord('p'):
                task_thread, task_client = create_task_client(target=path_plan_to_target,args=(target_name,),start_task=True)
            elif key & 0xFF == ord('r'):
                task_thread, task_client = create_task_client(target=rotate_path,start_task=True)
