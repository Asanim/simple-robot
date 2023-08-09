import cv2
import torch
import numpy as np
import greengrasssdk
import json

# Create Greengrass client
client = greengrasssdk.client('iot-data')

# Load YOLOv4 model
model = torch.hub.load('ultralytics/yolov5:v6.0', 'yolov4', pretrained=True)

# Kinect camera input
kinect_video_stream = cv2.VideoCapture(0)  # Change this for the Kinect v1 camera input

# YOLOv4 Inference function
def detect_objects(frame):
    # Convert to RGB and perform inference
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(img)
    return results

def publish_bounding_boxes(results):
    # Extract bounding boxes and class labels
    boxes = results.xywh[0].cpu().numpy()
    labels = results.names

    # Publish bounding boxes to Greengrass
    bounding_boxes = []
    for box in boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        label = labels[int(box[5])]
        bounding_boxes.append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'label': label})

    client.publish(
        topic='robot/bounding_boxes',
        payload=json.dumps(bounding_boxes)
    )

def process_kinect_frame():
    ret, frame = kinect_video_stream.read()
    if ret:
        results = detect_objects(frame)
        publish_bounding_boxes(results)

def main():
    while True:
        process_kinect_frame()

if __name__ == "__main__":
    main()
