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


import detection_utils as dt_util
from lidar_plotter import LidarPlotter

from queue import Queue

from simulation_tasks import viewer_task

DRONE_HEIGHT = -10

if __name__ == "__main__":
    
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Turn on detection
    detect_filter = "Monument*"
    dt_util.detection_filter_on_off(client, True, detect_filter)
    detect_filter = "Car*"
    dt_util.detection_filter_on_off(client, True, detect_filter)
    # detect_filter = "Deer*"
    # dt_util.detection_filter_on_off(client, True, detect_filter)
    # detect_filter = "Raccoon*"
    # dt_util.detection_filter_on_off(client, True, detect_filter)
    # detect_filter = "InstancedFoliageAct*"
    # detection_filter_on_off(client, True, detect_filter)

    # Create a thread to show the FPV feed
    png_queue = Queue()
    viewer_thread = threading.Thread(target=viewer_task, args=(DRONE_HEIGHT,png_queue))
    viewer_thread.start()

    while viewer_thread.is_alive():

        # Decode raw image 
        png = dt_util.get_fpv_frame(client=client)
 
        # Now run detect process
        detect_objects = dt_util.get_detected_object(client)
        if(detect_objects!=None):
            for object in detect_objects:
                dt_util.draw_object_detection(png,object)
            detect_list = detect_objects
        else:
            detect_list = []

        # Draw HUD
        dt_util.draw_HUD(png,client)

        # Push image into queue
        png_queue.put(png)

    # Cleanup
    print("Main thread cleaning up")
    cv2.destroyAllWindows() 
    client.enableApiControl(False)
    client.armDisarm(False)

    print("Main thread exiting")
    sys.exit(0)