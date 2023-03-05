import setup_path 
import airsim
import cv2
import numpy as np 
import pprint

def detection_filter_on_off(on_off):
    if on_off:
        # add desired object name to detect in wild card/regex format
        client.simAddDetectionFilterMeshName(camera_name, image_type, detect_filter_name) 
    else :
        client.simClearDetectionMeshNames(camera_name, image_type)

def get_distance_color(distance_tuple):
    # calculate the magnitude of the distance vector
    distance_mag = np.linalg.norm(distance_tuple)

    SCALE = 50.0

    # interpolate between red and green based on distance magnitude
    red = max(min(255 * (distance_mag / SCALE), 255), 0)
    green = max(min(255 * ((SCALE - distance_mag) /SCALE), 255), 0)

    return (0, int(green), int(red))

if __name__ == "__main__":
    # connect to the AirSim simulator
    client = airsim.VehicleClient()
    client.confirmConnection()

    # set camera name and image type to request images and detections
    camera_name = "0"
    image_type = airsim.ImageType.Scene
    detect_filter_name = "Monument*"

    # set detection radius in [cm]
    client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)
    detection_filter_on_off(True)

    # client.planPath

    while True:
        rawImage = client.simGetImage(camera_name, image_type)
        if not rawImage:
            continue
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        cars = client.simGetDetections(camera_name, image_type)
        if cars:
            for car in cars:
                if car.name == "Monument_01_176":
                    # s = pprint.pformat(car)
                    # print("car: %s" % s)
                    relative_position_vector = car.relative_pose.position
                    distance_tuple = (relative_position_vector.x_val,relative_position_vector.y_val,relative_position_vector.z_val)
                    
                    # draw bounding box
                    cv2.rectangle(png,(int(car.box2D.min.x_val),int(car.box2D.min.y_val)),(int(car.box2D.max.x_val),int(car.box2D.max.y_val)),(255,0,0),2)
                    
                    # draw car name
                    cv2.putText(png, car.name, (int(car.box2D.min.x_val),int(car.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), thickness=2)
                    
                    # draw distance text with color based on distance magnitude
                    distance_text = "Distance: %.2f" % np.linalg.norm(distance_tuple)
                    color = get_distance_color(distance_tuple)
                    cv2.putText(png, distance_text, (int(car.box2D.min.x_val),int(car.box2D.max.y_val + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness=2)

        cv2.imshow("AirSim", png)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif cv2.waitKey(1) & 0xFF == ord('c'):
            detection_filter_on_off(False)
        elif cv2.waitKey(1) & 0xFF == ord('a'):
            detection_filter_on_off(True)

    cv2.destroyAllWindows() 
