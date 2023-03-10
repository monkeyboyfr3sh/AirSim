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
    object_xmin, object_xmax = int(detect_object.box2D.min.x_val), int(detect_object.box2D.max.x_val)
    object_ymin, object_ymax = int(detect_object.box2D.min.y_val), int(detect_object.box2D.max.y_val)
    center_x = int((object_xmin + object_xmax) / 2)
    
    # draw bounding box
    cv2.rectangle(png,(object_xmin,object_ymin),(object_xmax,object_ymax),(255,0,0),2)
    # draw vertical line at the center of the bounding box
    cv2.line(png, (center_x, object_ymin), (center_x, object_ymax), (0, 255, 0), thickness=2)

    # draw detect_object name
    cv2.putText(png, detect_object.name, (int(detect_object.box2D.min.x_val),int(detect_object.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), thickness=2)

    # draw distance magnitude
    relative_position_vector = detect_object.relative_pose.position
    distance_text = "Distance: %.2f" % (relative_position_vector.get_length())
    cv2.putText(png, distance_text, (object_xmin, (object_ymax + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), thickness=2)

    return png



def detection_filter_on_off(client:airsim.MultirotorClient, on_off,detect_filter_name=None,detect_radius_cm=200,  camera_name = "0", image_type = airsim.ImageType.Scene):
    if on_off:
        # add desired object name to detect in wild card/regex format
        client.simSetDetectionFilterRadius(camera_name, image_type, detect_radius_cm * 100)
        client.simAddDetectionFilterMeshName(camera_name, image_type, detect_filter_name) 
    else :
        client.simClearDetectionMeshNames(camera_name, image_type)

def get_distance_color(distance_mag,scale = 50.0):
    # interpolate between red and green based on distance magnitude
    red = max(min(255 * (distance_mag / scale), 255), 0)
    green = max(min(255 * ((scale - distance_mag) /scale), 255), 0)
    return (0, int(green), int(red))