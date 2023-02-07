import setup_path
import airsim

import numpy as np
import cv2

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skspatial.objects import Sphere
import time

from custom_util.airsim_data_utils import AirSimClientManager

SCRIPT_TIME_SECONDS = 90
SPHERE_RADIUS = 2
input_message = "Now waiting to view data...\n" +\
                "1      - Continue sim for {} seconds, DO NOT delete data\n".format(SCRIPT_TIME_SECONDS) +\
                "2      - Continue sim for {} seconds, DO delete data\n".format(SCRIPT_TIME_SECONDS) +\
                "else   - Kill script\n"

def collision_handler(client_manager: AirSimClientManager):
    # First pause sim
    client_manager.simPause(True)

    # Handle the collision event
    print("Collision event!!!!")
    time.sleep(.1)

    # Now resume the sim
    client_manager.simPause(False)


def main():
    # Create plots
    fig = plt.figure(figsize=(6,10))
    time_ax = fig.add_subplot(2,1,1)
    spatial_ax = fig.add_subplot(2,1,2, projection='3d')

    # Create client manager
    client_manager = AirSimClientManager()

    # Now run script for set period
    script_start = time.time()
    client_manager.simPause(False)
    while ( True ):

        # Do a sampling round of the sensor data
        client_manager.sample_client_data()

        if(client_manager.collision_info.has_collided):
            collision_handler(client_manager)

        # Plot the new data point
        time_ax.scatter(client_manager.timestamp_list, client_manager.speed_list, c = client_manager.speed_list , cmap = "magma")
        spatial_ax.quiver(client_manager.position.x_val, client_manager.position.y_val, -client_manager.position.z_val, client_manager.velocity.x_val, client_manager.velocity.y_val, -client_manager.velocity.z_val)
        draw_sphere_radius = max(max(client_manager.speed_list),SPHERE_RADIUS)
        sphere = Sphere([client_manager.position.x_val, client_manager.position.y_val, -client_manager.position.z_val], draw_sphere_radius)
        sphere.plot_3d(spatial_ax, alpha=0.2)
        spatial_ax.scatter(client_manager.x_coord,client_manager.y_coord,client_manager.z_coord,c = client_manager.speed_list , cmap = "magma")

        # Set the axis labels
        time_ax.set_title('Speed vs Time')
        time_ax.set_xlabel('Time')
        time_ax.set_ylabel('Speed')
        
        spatial_ax.set_title('Position (heat map by speed)')
        spatial_ax.set_xlabel('x-coord')
        spatial_ax.set_ylabel('y-cood')
        spatial_ax.set_zlabel('z-coord')

        # Update plot and pause
        plt.draw() 
        plt.pause(0.1)

        # Check for sim runtime to elapse
        if ( (time.time()-script_start) > SCRIPT_TIME_SECONDS ):
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
            else:
                break

        # Clear the plot to plot new data points
        time_ax.clear()
        spatial_ax.clear()
        spatial_ax.clear()

if __name__=="__main__":
    main()
