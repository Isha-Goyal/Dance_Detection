"""
Uses pre-trained yolo model to determine pose in a single frame image.

input: img_path
note: adjust whether save is false or true
"""

from ultralytics import YOLO
import cv2
from time import time
import tensorflow as tf

# Load YOLOv8 pose estimation model
model = YOLO("yolov8m-pose.pt")

# Path to the input image
img_path = "test_imgs_vids/pose-test-img.jpg"

# Perform YOLOv8 inference on the input image
results = model(source=img_path, show=True, conf=0.3, save=False)

# Extract keypoints from the results
keypoints = results[0].keypoints.xy
keypoints = keypoints.cpu().numpy()  # Convert tensor to numpy array
print(keypoints)

# Read the input image for visualization
plotted_img = cv2.imread(img_path)

# Plot keypoints on the image
for pt in keypoints[0]:
    ctr = (int(pt[0]), int(pt[1]))
    print(pt)
    plotted_img = cv2.circle(
        plotted_img, ctr, radius=2, color=(0, 0, 255), thickness=-1
    )

# Display the image with plotted keypoints
cv2.imshow("plotted", plotted_img)
cv2.waitKey(0)
