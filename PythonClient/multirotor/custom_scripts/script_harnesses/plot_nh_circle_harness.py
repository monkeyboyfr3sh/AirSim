import get_harness_paths
import setup_path
import airsim

import numpy as np
import cv2

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skspatial.objects import Sphere

import threading

import numpy as np
import matplotlib.pylab as pl

from tasks.nh_circle import run_nh_circle_path
from util.airsim_data_utils import AirSimClientManager

SCRIPT_TIME_SECONDS = 0
DRONE_SPEED = 20
input_message = "Now waiting to view data...\n" +\
                "1      - Continue sim for {} seconds, DO NOT delete data\n".format(SCRIPT_TIME_SECONDS) +\
                "2      - Continue sim for {} seconds, DO delete data\n".format(SCRIPT_TIME_SECONDS) +\
                "3      - Continue sim for {} seconds, DO NOT delete data, start task again\n".format(SCRIPT_TIME_SECONDS) +\
                "4      - Continue sim for {} seconds, DO delete data, start task again\n".format(SCRIPT_TIME_SECONDS) +\
                "else   - Kill script\n"

def start_task_thread():
    task_thread = threading.Thread(target=run_nh_circle_path,args=(DRONE_SPEED,))
    task_thread.start()
    return task_thread

def main():
    # Create plots
    fig_1 = plt.figure(figsize=(10,6))
    time_ax_1 = fig_1.add_subplot(1,2,1)
    spatial_ax_1 = fig_1.add_subplot(1,2,2, projection='3d')

    # Create client manager
    client_manager = AirSimClientManager()

    # Now run script for set period
    script_start = time.time()
    client_manager.simPause(False)

    # Start task
    thread_dead_check = True
    task_thread = start_task_thread()

    while ( True ):

        # Do a sampling round of the sensor data
        client_manager.sample_client_data()

        # Plot the new speed data with heat map
        time_ax_1.scatter(client_manager.timestamp_list, client_manager.speed_list, c = client_manager.speed_list , cmap = "magma")
        time_ax_1.vlines(client_manager.collision_time_data_list,0,DRONE_SPEED,colors='red',linestyles='dashdot')

        # Now plot speed data onto spatial map
        spatial_ax_1.quiver(client_manager.position.x_val, client_manager.position.y_val, -client_manager.position.z_val, client_manager.velocity.x_val, client_manager.velocity.y_val, -client_manager.velocity.z_val)
        spatial_ax_1.scatter(client_manager.x_coord,client_manager.y_coord,client_manager.z_coord,c = client_manager.speed_list , cmap = "magma")
        spatial_ax_1.scatter(   client_manager.collision_x_data_list,
                                client_manager.collision_y_data_list,
                                client_manager.collision_z_data_list,
                                linewidths=10,c='red')

        # Set the axis labels
        time_ax_1.set_title('Speed vs Time')
        time_ax_1.set_xlabel('Time')
        time_ax_1.set_ylabel('Speed')
        
        spatial_ax_1.set_title('Position (heat map by speed)')
        spatial_ax_1.set_xlabel('x-coord')
        spatial_ax_1.set_ylabel('y-cood')
        spatial_ax_1.set_zlabel('z-coord')

        # Update plot and pause
        plt.draw() 
        plt.pause(0.1)

        # Check for sim runtime to elapse
        if  ( (time.time()-script_start) > SCRIPT_TIME_SECONDS and (not (SCRIPT_TIME_SECONDS==0)) ) or\
            ( (not ( task_thread.is_alive() )) and (thread_dead_check) ):
            
            # Pause the sim
            client_manager.simPause(True)
            
            # Get input on next step
            command = input(input_message)
            print('')

            # Now parse the input
            if (command == "1"):
                script_start = time.time() # update start to continue sim
                client_manager.simPause(False)
            elif (command == "2"):
                client_manager.init_client_data()
                script_start = time.time() # update start to continue sim
                client_manager.simPause(False)
            elif (command == "3"):
                script_start = time.time() # update start to continue sim
                client_manager.simPause(False)

                # Start task again
                thread_dead_check = True
                task_thread = start_task_thread()
            elif (command == "4"):
                client_manager.init_client_data()
                script_start = time.time() # update start to continue sim
                client_manager.simPause(False)

                # Start task again
                thread_dead_check = True
                task_thread = start_task_thread()
            else:
                client_manager.simPause(False)
                break
            
            # Prevent dead thread from entering again if it is already dead
            thread_dead_check = task_thread.is_alive()

        # Clear the plot to plot new data points
        time_ax_1.clear()
        spatial_ax_1.clear()

if __name__=="__main__":
    main()
