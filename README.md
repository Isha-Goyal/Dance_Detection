# Dancing Neatos:

### Maya Cranor and Isha Goyal

**Final project for Computational Robotics F'23**

We use a video or a livestream from a webcam of a person dancing to control a fleet of Neatos (vacuuming robots). Our algorithm determines the keypoints (such a joints and facial features) on the person in the video and tracks those movements. Then, our Neatos, which represent various joints, recreate it!

[Find out more on our project website!](https://sites.google.com/d/1ufQN_tTxO0oaJaZOAn-jLHr9FHY_Wo-w/p/1CYW6M2799S7LYBzcWnM-6ZxBDkFgoDmS/edit)

## Joint Detection - YOLO

In order to determine the movement of the person 'dancing,' we used an OpenCV model called YOLOv8, developed by Ultralytics. YOLOv8 is able to detect a person's joints, referred to as keypoints. We selected the nose, left/right wrist, and left/right ankle to be the five keypoints to track. At a high level, we feed the video (webcam or prerecorded video) into our model, and it outputs an array of keypoints for every frame. Conveniently, it can identify each keypoint and keep them consistent over time (for example, it recognizes the left shoulder in each frame and labels it accordingly). Every time YOLOv8 processes a frame, we extract the five tracked keypoints, and the data is formatted for publication to the movement node.

## Controlling the Neatos

Neatos are vacuuming robots, serving as a convenient platform for testing code when controlled by a Raspberry Pi. Within our fleet of five Neatos, each robot represents a keypoint, including the head, arms, and legs. To control each Neato and provide separate movement instructions, we created a movement publisher node correlated with a specific Neato. Additionally, our movement node had a subscriber linked to each Neato to obtain odometry information. To efficiently store Neato-specific information, we used a dictionary containing specific details for each Neato. When the movement node received the latest set of keypoints, it assigned the delta between the current position of the Neato, given by the odometry, and the position of the target keypoint. It then published the corresponding commands to each Neato to turn to the correct heading and move the correct distance.

## Initializing Neatos and Creating a Global Map

The webcam captures a grid of 650x500 pixels, which we decided to represent as a 3.25 x 2.5-meter frame for the Neatos. Every keypoint is converted to the map frame, and the received odometry of each Neato is also converted to the map frame. The code assumes that each Neato starts at the correct position and orientation. Upon receiving the first set of keypoints from the keypoint processing node, the location of each Neato on the global map is set to the corresponding tracked keypoint. As each Neato's location is tracked via odometry, the transformation from the global map to odometry is recorded for each Neato.

## Connecting the Neatos

To connect to the Neatos, use the following commands and adjust the IP address:

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.89 robot_name:=head udp_video_port:=5003 udp_sensor_port:=7778

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.90 robot_name:=left_a udp_video_port:=5004 udp_sensor_port:=7779

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.64 robot_name:=right_a udp_video_port:=5005 udp_sensor_port:=7777

ros2 launch neato_node2 bringup_multi.py host:=192.168.17.209 robot_name:=left_l udp_video_port:=5006 udp_sensor_port:=7776

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.64 robot_name:=right_l udp_video_port:=5007 udp_sensor_port:=7775
