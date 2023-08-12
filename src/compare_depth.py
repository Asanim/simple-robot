import numpy as np
import json
import greengrasssdk
import cv2

# Create Greengrass client
client = greengrasssdk.client('iot-data')

# Kinect depth stream (simulated)
def get_depth_map():
    # Example: Replace with actual Kinect depth map retrieval code
    depth_map = np.random.uniform(0, 10, (480, 640))  # Simulated depth map (in meters)
    return depth_map

def compare_bounding_boxes_with_depth(bounding_boxes, depth_map):
    proximity_data = []
    for box in bounding_boxes:
        x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
        depth = np.mean(depth_map[y1:y2, x1:x2])  # Mean depth within the bounding box region
        proximity_data.append({'label': box['label'], 'depth': depth})

    # Publish proximity data to Greengrass
    client.publish(
        topic='robot/proximity_data',
        payload=json.dumps(proximity_data)
    )

def process_bounding_boxes():
    # Assuming we have access to both the bounding boxes and the depth map
    bounding_boxes = []  # You will receive this from YOLO detection
    depth_map = get_depth_map()
    compare_bounding_boxes_with_depth(bounding_boxes, depth_map)

def main():
    while True:
        process_bounding_boxes()

if __name__ == "__main__":
    main()
