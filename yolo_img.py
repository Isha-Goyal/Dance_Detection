"""
Uses pre-trained yolo model to determine pose in a single frame image.

input: img_path
note: adjust whether save is false or true
"""


from ultralytics import YOLO
import torch
import numpy as np
import cv2
from time import time
import sys
import tensorflow as tf

model = YOLO("yolov8m-pose.pt")
img_path = "pose_test-img"
            
results = model(source=img_path, show=True, conf=0.3, save=False)
# results = results.tolist()

keypoints = results[0].keypoints.xy # what exactly does results[0] do for you? what are the other elements in results?
keypoints = keypoints.cpu().numpy() # convert tensor to numpy array
print(keypoints)

plotted_img = cv2.imread(img_path)

for pt in keypoints[0]:

    ctr = (int(pt[0]), int(pt[1]))
    print(pt)
    plotted_img = cv2.circle(plotted_img, ctr, radius=2, color=(0, 0, 255), thickness=-1)

cv2.imshow("plotted", plotted_img)
cv2.waitKey(0)