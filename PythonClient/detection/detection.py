import setup_path 
import airsim
import cv2
import numpy as np 
import pprint
import msgpackrpc
import future

import sys
import time

DRONE_HEIGHT = -3
DISTANCE_CLOSE = 10
DISTANCE_FAR = 20

class Direction:
    def __init__(self, num):
        if -0.12 <= num < 0.12:
            self.direction = 'north'
        elif 0.12 <= num < 0.55:
            self.direction = 'northeast'
        elif 0.55 <= num < 0.79:
            self.direction = 'east'
        elif 0.79 <= num <= 0.96:
            self.direction = 'southeast'
        elif 0.96 < num or -0.96 > num:
            self.direction = 'south'
        elif -0.55 <= num < -0.12:
            self.direction = 'northwest'
        elif -0.79 <= num < -0.55:
            self.direction = 'west'
        elif -0.96 <= num < -0.79:
            self.direction = 'southwest'
    def get_direction(self):
        return self.direction

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

def move_away_from_monument(client:airsim.MultirotorClient, z: int, monument_object: airsim.DetectionInfo, distance: int = DISTANCE_FAR):
    print("flying away from monument...")
    # Get the relative and current position of drone
    relative_position = monument_object.relative_pose.position
    client_position = client.simGetVehiclePose().position
    direction = Direction(orientation.z_val).get_direction()
    monument_position = client_position + relative_position

    # Now check facing of drone, update position accordingly
    # if east or west, offset y
    if (direction=='east') or (direction=='west'):
        new_x = monument_position.x_val
        new_y = monument_position.y_val-distance
    # if north or south, offset x
    if (direction=='north') or (direction=='south'):
        new_x = monument_position.x_val-distance
        new_y = monument_position.y_val

    job = client.moveToPositionAsync(   new_x, new_y, z, 2 )
    return job

def move_forward_to_monument(client:airsim.MultirotorClient, z: int, monument_object: airsim.DetectionInfo, distance: int = DISTANCE_CLOSE):
    print("flying forward to monument...")
    
    rotate_job = client.rotateByYawRateAsync(20,3.0)
    while(rotate_job.result==None):
        yaw = airsim.to_eularian_angles(
            monument_object.relative_pose.orientation
        )
        print(yaw,end="\r")
    # direction = Direction(orientation.z_val).get_direction()
    # monument_position = client_position + relative_position

    # # Now check facing of drone, update position accordingly
    # # if east or west, offset y
    # if (direction=='east') or (direction=='west'):
    #     new_x = monument_position.x_val
    #     new_y = monument_position.y_val-distance
    # # if north or south, offset x
    # if (direction=='north') or (direction=='south'):
    #     new_x = monument_position.x_val-distance
    #     new_y = monument_position.y_val

    # job = client.moveToPositionAsync(   new_x, new_y, z, 2 )
    # return job

def draw_HUD(png,client:airsim.MultirotorClient):
    
    # draw position in the bottom left corner
    position = client.simGetVehiclePose().position
    position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(position.x_val, position.y_val, position.z_val)
    cv2.putText(png, position_text, (20, png.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness=2)
    
    # draw orientation in the bottom left corner (Converted to direction)
    orientation = client.simGetVehiclePose().orientation
    # orientation_text = "Orientation: ({:.2f}, {:.2f}, {:.2f}) -> {}".format(orientation.x_val, orientation.y_val, orientation.z_val,Direction(orientation.z_val).direction)
    orientation_text = "Orientation: {}".format(Direction(orientation.z_val).get_direction())
    cv2.putText(png, orientation_text, (20, png.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness=2)

def get_detected_object(client:airsim.MultirotorClient, camera_name, image_type, detect_name) -> airsim.DetectionInfo:
    detect_objects = client.simGetDetections(camera_name, image_type)
    if detect_objects:
        for detect_object in detect_objects:
            if detect_object.name == detect_name:
                return detect_object
    return None

def draw_object_detection(png, detect_object: airsim.DetectionInfo):
    # draw bounding box
    cv2.rectangle(png,(int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val)),(int(detect_object.box2D.max.x_val),int(detect_object.box2D.max.y_val)),(255,0,0),2)
    
    # draw detect_object name
    cv2.putText(png, detect_object.name, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), thickness=2)
    
    # draw distance text with color based on distance magnitude
    relative_position_vector = detect_object.relative_pose.position
    distance_tuple = (relative_position_vector.x_val,relative_position_vector.y_val,relative_position_vector.z_val)
    distance_text = "Distance: %.2f (vector=<%.2f,%.2f,%.2f>)" % (relative_position_vector.get_length(),relative_position_vector.x_val,relative_position_vector.y_val, relative_position_vector.z_val)
    color = get_distance_color(distance_tuple,scale=20.0)
    # cv2.putText(png, distance_text, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.max.y_val + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness=2)
    cv2.putText(png, distance_text, (int(detect_object.box2D.min.x_val/2),int(detect_object.box2D.max.y_val + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), thickness=2)
    return png

if __name__ == "__main__":
    
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # set camera name and image type to request ima
    # ges and detections
    camera_name = "0"
    image_type = airsim.ImageType.Scene
    detect_filter_name = "Monument*"
    # Turn on detection
    detection_filter_on_off(True, detect_filter_name)

    #init vars
    z = DRONE_HEIGHT
    has_job = False
    job = None
    monument_object = None

    # client.armDisarm(True)
    job = client_disarm(client=client)
    has_job = True

    while True:

        # Get the raw image frame
        rawImage = client.simGetImage(camera_name, image_type)
        if not rawImage:
            continue
        # Decode raw image 
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)

        # Draw HUD
        draw_HUD(png,client)

        # Now run detect process
        monument_object = get_detected_object(client,camera_name,image_type,"Monument_01_176")
        if(monument_object!=None):
            draw_object_detection(png,monument_object)

        # Has a job that has just finished
        if (has_job and job.result!=None):
            print(f"job finised with result: {job.result}")
            has_job = False
        
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
            if(run_schedule):
                print("Enabling schedule")
            else:
                print("Disabling schedule")

    cv2.destroyAllWindows() 
    client.armDisarm(False)

