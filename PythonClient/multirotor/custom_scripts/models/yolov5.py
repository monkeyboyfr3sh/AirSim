import torch
import cv2

def draw_boxes(img, results, confidence_threshold=0.5):
    class_names = results.names  # Get class names from the model
    for *xyxy, conf, cls in results.xyxy[0]:
        if conf >= confidence_threshold:
            class_name = class_names[int(cls)]
            cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
            cv2.putText(img, f'{class_name} {conf:.2f}', (int(xyxy[0]), int(xyxy[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img

# Model - we will use yolov5s
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True).to("cuda")

# Image
img_path = "C:\\Users\\david\\Downloads\\Taka_Shiba.jpg"
img = cv2.imread(img_path)

# Inference
results = model(img)

# Draw bounding boxes
img_with_boxes = draw_boxes(img, results)

# Display image
cv2.imshow('Image', img_with_boxes)
cv2.waitKey(0)
cv2.destroyAllWindows()
