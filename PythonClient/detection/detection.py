import setup_path 
import airsim
import cv2
import numpy as np 
import pprint

import sys
import time

def detection_filter_on_off(on_off,detect_filter_name=None,detect_radius_cm=200):
    if on_off:
        # add desired object name to detect in wild card/regex format
        client.simSetDetectionFilterRadius(camera_name, image_type, detect_radius_cm * 100)
        client.simAddDetectionFilterMeshName(camera_name, image_type, detect_filter_name) 
    else :
        client.simClearDetectionMeshNames(camera_name, image_type)

def get_distance_color(distance_tuple,scale = 50.0):
    # calculate the magnitude of the distance vector
    distance_mag = np.linalg.norm(distance_tuple)
    # interpolate between red and green based on distance magnitude
    red = max(min(255 * (distance_mag / scale), 255), 0)
    green = max(min(255 * ((scale - distance_mag) /scale), 255), 0)
    return (0, int(green), int(red))

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
    client.moveToZAsync(z, 1).join()

MOVE_SCHEDULE_SECONDS = 5
DRONE_HEIGHT = -3

if __name__ == "__main__":
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    print("arming the drone...")
    client.armDisarm(True)

    z = DRONE_HEIGHT
    client_takeoff(client=client,z=z)

    # set camera name and image type to request ima
    # ges and detections
    camera_name = "0"
    image_type = airsim.ImageType.Scene
    detect_filter_name = "Monument*"
    # Turn on detection
    detection_filter_on_off(True, detect_filter_name)

    schedule_timestamp = time.time()
    while True:
        # Get the raw image frame
        rawImage = client.simGetImage(camera_name, image_type)
        if not rawImage:
            continue
        # Decode raw image 
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        
        # Now run detect process
        detect_objects = client.simGetDetections(camera_name, image_type)
        if detect_objects:
            for detect_object in detect_objects:
                if detect_object.name == "Monument_01_176":

                    # draw bounding box
                    cv2.rectangle(png,(int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val)),(int(detect_object.box2D.max.x_val),int(detect_object.box2D.max.y_val)),(255,0,0),2)
                    
                    # draw detect_object name
                    cv2.putText(png, detect_object.name, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), thickness=2)
                    
                    # draw distance text with color based on distance magnitude
                    relative_position_vector = detect_object.relative_pose.position
                    distance_tuple = (relative_position_vector.x_val,relative_position_vector.y_val,relative_position_vector.z_val)
                    # distance_text = "Distance: %.2f" % np.linalg.norm(distance_tuple)
                    distance_text = "Distance: %.2f" % relative_position_vector.get_length()
                    color = get_distance_color(distance_tuple)
                    cv2.putText(png, distance_text, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.max.y_val + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness=2)

        # Show image and check input
        cv2.imshow("AirSim", png)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('c'):
            detection_filter_on_off(False,detect_filter_name)
        elif key & 0xFF == ord('f'):
            detection_filter_on_off(True,detect_filter_name)
        elif key & 0xFF == ord('d'):
            client.moveToZAsync(0, 1).join()
            client.enableApiControl(False)
        elif key & 0xFF == ord('t'):
            client.enableApiControl(True)
            client_takeoff(client=client,z=z)

        # # Check for time to schedule a flight path
        # if( (time.time()-schedule_timestamp) > MOVE_SCHEDULE_SECONDS ):
        #     # Update timestamp
        #     schedule_timestamp = time.time()
        #     # Need to schedule a flight forward 
        #     if(distance_text > 3):
        #         client.simGetVehiclePose().position

        #         path =  [   airsim.Vector3r(125,0,z),
        #                     airsim.Vector3r(125,-130,z),
        #                     airsim.Vector3r(0,-130,z),
        #                     airsim.Vector3r(0,0,z) ]
        #     # Need to schedule a flight backward
        #     else:
                
        #     # Schedule the flight path
        #     print("flying on path...")
        #     result = client.moveOnPathAsync(
        #                             path=path,
        #                             velocity=12, timeout_sec=120,
        #                             drivetrain=airsim.DrivetrainType.ForwardOnly,
        #                             yaw_mode=airsim.YawMode(False,0),
        #                             lookahead=20, adaptive_lookahead=1 ).join()

    client.armDisarm(False)
    cv2.destroyAllWindows() 
