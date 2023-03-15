import setup_path 
import airsim
import cv2
import numpy as np 

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import sys
import time
import threading

from detection_utils import Direction, draw_HUD, draw_object_detection, get_detected_object, get_fpv_frame, detection_filter_on_off,\
                            move_to_distance_from_object, center_on_detection

from lidar_plotter import lidar_plotter

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
                            velocity=5, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=20, adaptive_lookahead=1 ).join()
    print("Starting path 2...",end=' ')
    client.moveOnPathAsync(
                            path=path_2,
                            velocity=5, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=10, adaptive_lookahead=1 ).join()
    print("Starting path 3...",end=' ')
    client.moveOnPathAsync(
                            path=path_3,
                            velocity=5, timeout_sec=120,
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
    center_on_detection(client,monument_name)
    move_to_distance_from_object(client,monument_name,z,distance, velocity = (2.5, 0.0, 0.0))
    # Center the drone 
    center_on_detection(client,monument_name)

    client_disarm(client=client)
    print('Navigation complete!')

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

    TARGET_NAME = "Monument_01_176"

    # Disarm client
    task_thread, task_client = create_task_client(target=client_disarm,start_task=False)
    task_thread.start()

    while (True):
        # Check if the queue has a new image
        if( not (png_queue.empty()) ):
            # Get the picture from queue
            png = png_queue.get()

            # Show image and take in user input
            cv2.imshow("AirSim", png)

        # Always poll the key stroke        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

        # Previous thread is done, can start a new task
        if not (task_thread.is_alive()):
            if key & 0xFF == ord('d'):
                task_thread, task_client = create_task_client(target=client_disarm,start_task=False)
                task_thread.start()
            elif key & 0xFF == ord('t'):
                task_thread, task_client = create_task_client(target=client_takeoff,args=(z,),start_task=False)
                task_thread.start()
            elif key & 0xFF == ord('n'):
                task_thread, task_client = create_task_client(target=navigate_to_monument,args=(z,),start_task=False)
                task_thread.start()
            elif key & 0xFF == ord('m'):
                task_thread, task_client = create_task_client(target=move_distance_from_monument,args=(z,TARGET_NAME, 10.0,5.0),start_task=False)
                task_thread.start()
