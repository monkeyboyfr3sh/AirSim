import setup_path 
import airsim
import cv2
import numpy as np

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

def get_fpv_frame(client:airsim.MultirotorClient, camera_name = "0", image_type = airsim.ImageType.Scene):
    rawImage = client.simGetImage(camera_name, image_type)
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    return png

def draw_HUD(png,client:airsim.MultirotorClient):

    # draw position in the bottom left corner
    position = client.simGetVehiclePose().position
    position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(position.x_val, position.y_val, position.z_val)
    cv2.putText(png, position_text, (20, png.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness=2)

    # draw orientation in the bottom left corner (Converted to direction)
    orientation = client.simGetVehiclePose().orientation
    orientation_text = "Orientation: {}".format(Direction(orientation.z_val).get_direction())
    cv2.putText(png, orientation_text, (20, png.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness=2)

    # draw cross in the center of the frame
    center_x = int(png.shape[1] / 2)
    center_y = int(png.shape[0] / 2)
    cv2.line(png, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), thickness=2)
    cv2.line(png, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), thickness=2)


def get_detected_object(client:airsim.MultirotorClient, detect_name, camera_name = "0", image_type = airsim.ImageType.Scene) -> airsim.DetectionInfo:
    detect_objects = client.simGetDetections(camera_name, image_type)
    if detect_objects:
        for detect_object in detect_objects:
            if detect_object.name == detect_name:
                return detect_object
    return None

def draw_object_detection(png, detect_object: airsim.DetectionInfo):
    
    # Get info about the object in frame position
    object_xmin, object_xmax = int(detect_object.box2D.min.x_val), int(detect_object.box2D.max.x_val)
    object_ymin, object_ymax = int(detect_object.box2D.min.y_val), int(detect_object.box2D.max.y_val)
    
    # Now get center for frame and object
    frame_center_x = int(png.shape[1] / 2)
    object_center_x = int((object_xmin + object_xmax) / 2)
    distance_from_center = abs(object_center_x - frame_center_x)

    # draw bounding box
    cv2.rectangle(png,(object_xmin,object_ymin),(object_xmax,object_ymax),(255,0,0),2)
    # draw vertical line at the center of the bounding box
    cv2.line(png, (object_center_x, object_ymin), (object_center_x, object_ymax), (0, 255, 0), thickness=2)

    # draw detect_object name
    cv2.putText(png, detect_object.name, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), thickness=2)

    # draw distance magnitude
    relative_position_vector = detect_object.relative_pose.position
    distance_text = "Distance: %.2f" % (relative_position_vector.get_length())
    cv2.putText(png, distance_text, (object_xmin, (object_ymax + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), thickness=2)

    # draw offcenterness of the object
    offcenter_text = "Offcenter: %.2f" % (distance_from_center)
    cv2.putText(png, offcenter_text, (object_xmin, (object_ymax + 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), thickness=2)

    return png

def detection_filter_on_off(client:airsim.MultirotorClient, on_off,detect_filter_name=None,detect_radius_m=50,  camera_name = "0", image_type = airsim.ImageType.Scene):
    if on_off:
        # add desired object name to detect in wild card/regex format
        client.simSetDetectionFilterRadius(camera_name, image_type, detect_radius_m * 100)
        client.simAddDetectionFilterMeshName(camera_name, image_type, detect_filter_name) 
    else :
        client.simClearDetectionMeshNames(camera_name, image_type)

def get_distance_color(distance_mag,scale = 50.0):
    # interpolate between red and green based on distance magnitude
    red = max(min(255 * (distance_mag / scale), 255), 0)
    green = max(min(255 * ((scale - distance_mag) /scale), 255), 0)
    return (0, int(green), int(red))

def center_on_detection(client:airsim.MultirotorClient, detect_name, base_yaw_rate=10,center_thresh=10,time_unit=0.1,max_spin_rate=40):
    detection_present = True

    # Stay in loop while object is not centered    
    while( True ):  
        png = get_fpv_frame(client=client)

        # Check if object is detect in frame
        detect_object = get_detected_object(client,detect_name)
        if(detect_object!=None):

            if not (detection_present):
                detection_present = True
                print(f"\nFound {detect_name}!")
            
            # Get info about the object in frame position
            object_xmin, object_xmax = int(detect_object.box2D.min.x_val), int(detect_object.box2D.max.x_val)
            object_ymin, object_ymax = int(detect_object.box2D.min.y_val), int(detect_object.box2D.max.y_val)
            
            # Now get center for frame and object
            frame_center_x = int(png.shape[1] / 2)
            object_center_x = int((object_xmin + object_xmax) / 2)
            distance_from_center = object_center_x - frame_center_x

            # Get the current center offset
            print(f"Distance from center = {distance_from_center}",end='\r')
            if( abs(distance_from_center) < center_thresh ):
                print("Offcenter goal reached")
                break

            # Need to rotate with positive yaw
            if(distance_from_center > 0):
                spin_rate = base_yaw_rate*1.0
                spin_rate = min(spin_rate,max_spin_rate)
                client.rotateByYawRateAsync(spin_rate,time_unit).join()
            # Need to rotate with negative yaw
            else:
                spin_rate = base_yaw_rate*1.0
                spin_rate = min(spin_rate,max_spin_rate)
                client.rotateByYawRateAsync(-spin_rate,time_unit).join()

        # Just spin if no object detected
        else:
            spin_rate = base_yaw_rate*5.0
            spin_rate = min(spin_rate,max_spin_rate)
            print(f"Searching for {detect_name}... (spin_rate:{spin_rate})",end='\r')
            client.rotateByYawRateAsync(spin_rate,time_unit).join()
            detection_present = False
    
    detect_object = get_detected_object(client,detect_name)
    # Now get center for frame and object
    frame_center_x = int(png.shape[1] / 2)
    object_center_x = int((object_xmin + object_xmax) / 2)
    distance_from_center = object_center_x - frame_center_x
    
    # Make sure client is hovering at the end 
    client.hoverAsync().join()

    print(f"\nfinal offset = {distance_from_center}")

def move_to_distance_from_object(client:airsim.MultirotorClient, detect_name: str, z: int, distance_goal: float, velocity: tuple = (1.0, 0.0, 0.0), distance_thresh: float = 0.1, time_unit: float = 0.1):
    # Now move closer to object until distance is desired
    while( True ):  

        # Check if object is detect in frame
        detect_object = get_detected_object(client,detect_name)
        if(detect_object!=None):
            
            # Get distance from object
            relative_position_vector = detect_object.relative_pose.position
            object_distance = relative_position_vector.get_length()

            # Get the current center offset
            print(f"Distance from object = {object_distance}",end='\r')
            if( ( (distance_goal-distance_thresh) <= object_distance ) and
                ( object_distance <= (distance_goal+distance_thresh) ) ):
                print("distance goal reached")
                break

            # Need to move towards object
            if(object_distance > distance_goal):
                client.moveByVelocityBodyFrameAsync(velocity[0], velocity[1], velocity[2], time_unit).join()
            # Need to move away from object
            else:
                client.moveByVelocityBodyFrameAsync(-velocity[0], velocity[1], velocity[2], time_unit).join()

    # Now hove at distance
    client.hoverAsync().join()
    client.moveToZAsync(z,1).join()

    # Get distance from object
    relative_position_vector = detect_object.relative_pose.position
    object_distance = relative_position_vector.get_length()
    print(f"\nfinal distance = {object_distance}")