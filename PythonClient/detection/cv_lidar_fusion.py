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

from lidar_plotter import lidar_plotter, parse_lidarData

DRONE_HEIGHT = -3
TARGET_NAME = "Monument_01_176"

def client_takeoff(client:airsim.MultirotorClient,z: int):
    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        client.hoverAsync().join()

    time.sleep(0.5)

    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("take off failed...")
        sys.exit(1)

    # AirSim uses NED coordinates so negative axis is up.
    # z of -5 is 5 meters above the original launch point.
    print("make sure we are hovering at {} meters...".format(-z))
    return client.moveToZAsync(z, 1)

def client_disarm(client:airsim.MultirotorClient):
    job = client.moveToZAsync(0, 1)
    job.join()
    client.enableApiControl(False)
    return job

def navigate_to_monument(client:airsim.MultirotorClient,z: int):
    # Schedule the flight path
    print("flying to monument...")
    path = [ airsim.Vector3r(127,-1.75,z),
             airsim.Vector3r(127,130,z),
             airsim.Vector3r(110,128,z),
             airsim.Vector3r(110,155,z) ]
    return client.moveOnPathAsync(
                            path=path,
                            velocity=5, timeout_sec=120,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False,0),
                            lookahead=10, adaptive_lookahead=1 )

def move_distance_from_monument(client:airsim.MultirotorClient, z: int, monument_name: str, distance: int ):
    print("flying to monument...")

    # First center the drone 
    center_on_detection(client,monument_name)
    move_to_distance_from_object(client,monument_name,z)

if __name__ == "__main__":
    
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    lidar_plot = lidar_plotter()

    # Turn on detection
    detect_filter_name = "Monument*"
    detection_filter_on_off(client, True, detect_filter_name)

    #init vars
    z = DRONE_HEIGHT

    while True:

        # Decode raw image 
        png = get_fpv_frame(client=client)

        # Now run detect process
        monument_object = get_detected_object(client,TARGET_NAME)
        if(monument_object!=None):
            draw_object_detection(png,monument_object)
            detect_list = [monument_object]
        else:
            detect_list = []
            
        # Get Lidar data
        lidarData = client.getLidarData()
        client_height = client.simGetVehiclePose().position.z_val
        points = parse_lidarData(lidarData,client_height)
        
        # Update the plot
        lidar_plot.update_plot(points,client,detect_list,pause_time=0.01)

        # Draw HUDq
        draw_HUD(png,client)

        # Show image and take in user input
        cv2.imshow("AirSim", png)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('d'):
            client.moveToZAsync(0, 1).join()
            client.enableApiControl(False)
        elif key & 0xFF == ord('t'):
            client.enableApiControl(True)
            client_takeoff(client=client,z=z)
        elif key & 0xFF == ord('n'):
            task_client = airsim.MultirotorClient()
            task_client.confirmConnection()
            task_thread = threading.Thread(target=navigate_to_monument, args=(task_client,z))
            task_thread.start()
        elif key & 0xFF == ord('y'):
            task_client = airsim.MultirotorClient()
            task_client.confirmConnection()
            task_thread = threading.Thread(target=move_distance_from_monument, args=(task_client,z,TARGET_NAME, 10))
            task_thread.start()
        elif key & 0xFF == ord('j'):
            # Create a client for the tasks to share
            task_client = airsim.MultirotorClient()
            task_client.confirmConnection()
            # Create thread for simple and cv navigation
            simple_nav_thread = threading.Thread(target=navigate_to_monument, args=(task_client,z))
            cv_nac=v_thread = threading.Thread(target=move_distance_from_monument, args=(task_client,z,TARGET_NAME, 10))

        elif key & 0xFF == ord('v'):
            client.rotateByYawRateAsync(20,1).join()

    cv2.destroyAllWindows() 
    client.enableApiControl(False)
    client.armDisarm(False)
