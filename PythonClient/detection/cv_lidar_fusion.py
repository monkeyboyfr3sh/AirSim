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

from simulation_tasks import viewer_task

DRONE_HEIGHT = -3
DISTANCE_CLOSE = 10
DISTANCE_FAR = 20

if __name__ == "__main__":
    
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Create a plotter for lidar data
    lidar_plot = lidar_plotter()

    # Turn on detection
    detect_filter = "Monument*"
    detection_filter_on_off(client, True, detect_filter)
    detect_filter = "Car*"
    detection_filter_on_off(client, True, detect_filter)
    detect_filter = "Deer*"
    detection_filter_on_off(client, True, detect_filter)
    detect_filter = "Raccoon*"
    detection_filter_on_off(client, True, detect_filter)

    # Create a thread to show the FPV feed
    png_queue = Queue()
    viewer_thread = threading.Thread(target=viewer_task, args=(DRONE_HEIGHT,png_queue))
    viewer_thread.start()

    while viewer_thread.is_alive():

        # Decode raw image 
        png = get_fpv_frame(client=client)

        # Now run detect process
        detect_objects = get_detected_object(client)
        if(detect_objects!=None):
            for object in detect_objects:
                draw_object_detection(png,object)
            detect_list = detect_objects
        else:
            detect_list = []
            
        # Get Lidar data
        lidarData = client.getLidarData()
        points = lidar_plot.parse_lidarData(lidarData,point_value_cap=50)

        # Update the plot
        lidar_plot.update_plot(points,client,detect_list,pause_time=0.01)

        # Draw HUDq
        draw_HUD(png,client)

        # Push image into queue
        png_queue.put(png)

    # Cleanup
    print("Main thread cleaning up")
    cv2.destroyAllWindows() 
    client.enableApiControl(False)
    client.armDisarm(False)

    print("Main thread exiting")
    sys.exit(0)