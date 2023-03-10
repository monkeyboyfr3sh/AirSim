import setup_path 
import airsim
import cv2
import numpy as np 
import pprint
import msgpackrpc
import future

import sys
import time

from detection_utils import Direction, draw_HUD, draw_object_detection, get_detected_object, get_fpv_frame, detection_filter_on_off

DRONE_HEIGHT = -3
DISTANCE_CLOSE = 10
DISTANCE_FAR = 20

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

def center_on_detection(client:airsim.MultirotorClient, detect_name, yaw_rate=20,yaw_thresh=0.05,time_unit=1):
    # Stay in loop while object is not centered    
    while( True ):  

        # Check if object is detect in frame
        detect_object = get_detected_object(client,detect_name)
        if(detect_object!=None):

            # Get the current offset yaw, break if it is smaller than thresh
            yaw_delta = airsim.to_eularian_angles( detect_object.relative_pose.orientation )[2]
            print(f"Current yaw = {yaw_delta}",end='\r')
            if( abs(yaw_delta) < yaw_thresh ):
                break

            # Need to rotate with positive yaw
            if(yaw_delta < 0):
                client.rotateByYawRateAsync(yaw_rate,time_unit).join()
            # Need to rotate with negative yaw
            else:
                client.rotateByYawRateAsync(-yaw_rate,time_unit).join()

        # Just spin if no object detected
        # FIXME: should probably have timeout or something
        else:
            client.rotateByYawRateAsync(yaw_rate,time_unit).join()

    print(f"\nfinal yaw = {yaw_delta}")

def move_forward_to_monument(client:airsim.MultirotorClient, z: int, monument_object: airsim.DetectionInfo, distance: int = DISTANCE_CLOSE):
    print("flying forward to monument...")

    center_on_detection(client,monument_object.name)

    rotate_job = client.rotateByYawRateAsync(20,1.0)
    while (rotate_job.result==None):
        orientation = client.simGetVehiclePose().orientation
        client_yaw = orientation.z_val
        print(f"Client yaw: {round(client_yaw,2)} ",end=' ')

        monument_object = get_detected_object(client,"Monument_01_176")
        if(monument_object!=None):
            draw_object_detection(png,monument_object)
            monument_yaw = airsim.to_eularian_angles( monument_object.relative_pose.orientation )[2]
            print(f"Monument yaw: {round(monument_yaw,2)}",end='')
        
        print('\r')

if __name__ == "__main__":
    
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # Turn on detection
    detect_filter_name = "Monument*"
    detection_filter_on_off(client, True, detect_filter_name)

    #init vars
    z = DRONE_HEIGHT
    monument_object = None
    job = None
    has_job = False

    while True:
        # Decode raw image 
        png = get_fpv_frame(client=client)

        # Now run detect process
        monument_object = get_detected_object(client,"Monument_01_176")
        if(monument_object!=None):
            draw_object_detection(png,monument_object)

        # Has a job that has just finished
        if (has_job and job.result!=None):
            print(f"job finised with result: {job.result}")
            has_job = False
        
        # Draw HUD
        draw_HUD(png,client)

        # Show image and take in user input
        cv2.imshow("AirSim", png)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('u'):
            detection_filter_on_off(False,detect_filter_name)
        elif key & 0xFF == ord('f'):
            detection_filter_on_off(True,detect_filter_name)
        elif key & 0xFF == ord('d'):
            client.moveToZAsync(0, 1).join()
            client.enableApiControl(False)
        elif key & 0xFF == ord('t'):
            client.enableApiControl(True)
            job = client_takeoff(client=client,z=z)
            has_job = True
        elif key & 0xFF == ord('n'):
            job = navigate_to_monument(client=client,z=z)
            has_job = True
        elif key & 0xFF == ord('y'):
            move_forward_to_monument(client=client,z=z,monument_object=monument_object)
            # job = move_forward_to_monument(client=client,z=z,monument_object=monument_object)
            # has_job = True
        elif key & 0xFF == ord('s'):
            run_schedule = not(run_schedule)

    cv2.destroyAllWindows() 
    client.armDisarm(False)
