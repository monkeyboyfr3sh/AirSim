import get_task_paths
import setup_path
import airsim

import torch
import cv2
import time
import sys
from enum import Enum

def draw_boxes(img, results, confidence_threshold=0.4):
    class_names = results.names  # Get class names from the model
    for *xyxy, conf, cls in results.xyxy[0]:
        if conf >= confidence_threshold:
            class_name = class_names[int(cls)]
            cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
            cv2.putText(img, f'{class_name} {conf:.2f}', (int(xyxy[0]), int(xyxy[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img

def get_off_center_boxes_for_class(results, class_name, confidence_threshold=0.5, max_center_deviation=10):
    class_names = results.names
    off_center_boxes = []
    img_h, img_w = results.imgs[0].shape[:2]
    img_center = (img_w // 2, img_h // 2)
    for *xyxy, conf, cls in results.xyxy[0]:
        if conf >= confidence_threshold and class_names[int(cls)] == class_name:
            box_center = ((xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2)
            if abs(box_center[0] - img_center[0]) > max_center_deviation or abs(box_center[1] - img_center[1]) > max_center_deviation:
                off_center_boxes.append( (box_center[0] - img_center[0], box_center[1] - img_center[1]) )
    return off_center_boxes

def update_yaw_with_offset_box(offset_pixel_count, current_yaw,yaw_offset_max=2):
    # Do nothing if basically locked in
    if (-20<offset_pixel_count) and (offset_pixel_count<20):
        return current_yaw
    # Now get offset if supposed to do something
    offset = offset_pixel_count/10 if (offset_pixel_count/10<yaw_offset_max) else yaw_offset_max
    # Need to + yaw
    if(offset_pixel_count>0):
        yaw = current_yaw + abs(offset)
    else:
        yaw = current_yaw - abs(offset)

class Direction(Enum):
    THROTTLE_UP = 0
    THROTTLE_DOWN = 1
    PITCH_FORWARD = 2
    PITCH_BACKWARD = 3
    ROLL_LEFT = 4
    ROLL_RIGHT = 5
    YAW_LEFT = 6
    YAW_RIGHT = 7

def run_yolo_rc_path(cameraType):
    
    # Check if a GPU is available
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Model - we will use yolov5s
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True).to(device)

    # Select camara type
    cameraTypeMap = {
    "depth": airsim.ImageType.DepthVis,
    "segmentation": airsim.ImageType.Segmentation,
    "seg": airsim.ImageType.Segmentation,
    "scene": airsim.ImageType.Scene,
    "disparity": airsim.ImageType.DisparityNormalized,
    "normals": airsim.ImageType.SurfaceNormals
    }

    # Connect to client
    print("Connecting to client")
    client = airsim.MultirotorClient()
    print("Confirming connection")
    client.confirmConnection()

    # Prep frame writes
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    thickness = 2
    textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
    print(textSize)
    textOrg = (10, 10 + textSize[1])
    frameCount = 0
    startTime = time.time()
    fps = 0

    detect_car = False
    rotate_drone = False
    active_control = False
    yaw_offset_max = 2
    yaw = 0
    rotate_period_seconds = 0.1
    rotate_timestamp = time.time()

    # Start CV task loop
    while True:

        if (not detect_car) and (rotate_drone) and (active_control) and ( (time.time()-rotate_timestamp) > rotate_period_seconds):
            rotate_timestamp = time.time()
            
            off_center_car_boxes = get_off_center_boxes_for_class(results=results,class_name="car")
            if (len(off_center_car_boxes)>0):
                car_box = off_center_car_boxes[0]
                pixel_offset = int(car_box[0].cpu().item())
                yaw = update_yaw_with_offset_box(pixel_offset,yaw)
                client.rotateToYawAsync(yaw)

        # Get the camera frame
        rawImage = client.simGetImage("0", cameraTypeMap[cameraType])
        if (rawImage == None):
            print("Camera is not returning image, please check airsim for error messages")
            sys.exit(0)
        else:
            img = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
            # Inference
            results = model(img) # pass the image through our model

            img_with_boxes = draw_boxes(img, results)
            
            cv2.putText(img_with_boxes,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,0,255),thickness)
            cv2.imshow("Depth", img_with_boxes)

        # Calaculate frame rate
        frameCount = frameCount  + 1
        endTime = time.time()
        diff = endTime - startTime
        if (diff > 1):
            fps = frameCount
            frameCount = 0
            startTime = endTime

        # Wait for input to kill script
        key = cv2.waitKey(1) & 0xFF
        if (key == 27 or key == ord('q') or key == ord('x')):
            print("Disabling api control")
            client.enableApiControl(False)
            break
        elif (key == ord(' ')):
            print("forcing yaw offset")
            yaw += yaw_offset_max
            client.rotateToYawAsync(yaw)
            rotate_timestamp = time.time()
        # Take control of sim
        elif (key == ord('e')):
            print("Enabling api control")
            client.enableApiControl(True)
            active_control = True
        # Give control of sim
        elif (key == ord('d')):
            print("Disabling api control")
            client.enableApiControl(False)
            active_control = False
        # Reset position of the drone
        elif (key == ord('r')):
            # Set position of drone
            pose = client.simGetVehiclePose()
            pose.position.x_val = 0
            pose.position.y_val = 0
            pose.position.z_val = 0
            client.simSetVehiclePose(pose, True)

        elif (key == ord('y')):
            rotate_drone = not rotate_drone
            if(rotate_drone):
                print("Starting rotation")
            else:
                print("Stopping rotation")
                
        elif (key == ord('t')):
            if not active_control:
                client.enableApiControl(True)
                active_control = True

            state = client.getMultirotorState()
            if state.landed_state == airsim.LandedState.Landed:
                print("taking off...")
                client.takeoffAsync().join()
            else:
                client.hoverAsync().join()

            # AirSim uses NED coordinates so negative axis is up.
            # z of -5 is 5 meters above the original launch point.
            z_base = -5
            print("make sure we are hovering at {} meters...".format(-z_base))
            client.moveToZAsync(z_base, 1).join()

        elif (key == ord('g')):            
            off_center_car_boxes = get_off_center_boxes_for_class(results=results,class_name="car")
            if (len(off_center_car_boxes)==0):
                print("Received no car boxes")
            for car_box in off_center_car_boxes:
                print(car_box)
            


if __name__ == "__main__":
    run_yolo_rc_path("scene")