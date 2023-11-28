from ultralytics import YOLO
# import torch
import numpy as np
import cv2
# from time import time
# import sys

model = YOLO("yolov8m-pose.pt")

# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Error reading video file")
#     sys.exit()

# while cap.isOpened():
#     success, frame = cap.read()
#     if success:
#         results = model.predict(frame)
#         boxes = results[0].boxes.xywh.cpu()
#         clss = results[0].boxes.cls.cpu().tolist()
#         names = results[0].names
#         for box, cls in zip(boxes, clss):
#             x, y, w, h = box
#             label = str(names[int(cls)])
            
results = model(source="pose-test-img.jpg", show=True, conf=0.3, save=False)

keypoints = results[0].keypoints.xy
keypoints = np.array(keypoints)
print(keypoints)

plotted_img = "pose-test-img.jpg"
for pt in keypoints:
    print(pt)
    plotted_img = cv2.circle(plotted_img, pt, radius=2, color=(0, 0, 255), thickness=-1)