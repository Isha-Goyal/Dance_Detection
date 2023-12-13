"""
Uses pre-trained yolo model to determine pose in a video.

input: webcam
note: showing annotated_frame will show the default visual that comes with the model (with all the colors and lines).
    Showing plotted_img will show the visual we are using for info gathering.
"""

import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from std_msgs.msg import Int32MultiArray


class KeyPointsNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        self.keypoint_pub = self.create_publisher(Int32MultiArray, "keypoint", 10)
        # Load the YOLOv8 model
        self.model = YOLO("yolov8m-pose.pt")
        # Open the video file
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.run_loop)

    def find_keypoints(self):
        """
        Loop through video frames, run YOLOv8 inference, visualize keypoints, and publish keypoint information.

        This method iterates through video frames captured by a webcam. For each frame, it performs YOLOv8 inference
        using a pre-trained model (self.model). The keypoints are extracted from the inference results and
        visualized on the frame. The keypoint information is published using a ROS publisher (self.keypoint_pub).
        The loop continues until 'q' is pressed or the end of the video is reached.

        """
        # Loop through the video frames
        while self.cap.isOpened():
            # Read a frame from the video
            success, frame = self.cap.read()

            if success:
                # Run YOLOv8 inference on the frame
                results = self.model(frame)

                # Visualize the results on the frame
                annotated_frame = results[0].plot()

                # Extract keypoints
                keypoints = results[0].keypoints.xy.cpu().numpy()

                # Publish keypoints using ROS
                int32_multi_array = Int32MultiArray()
                int32_multi_array.data = [
                    int(keypoints[0][0][0]),
                    int(keypoints[0][0][1]),
                    int(keypoints[0][9][0]),
                    int(keypoints[0][9][1]),
                    int(keypoints[0][10][0]),
                    int(keypoints[0][10][1]),
                    int(keypoints[0][15][0]),
                    int(keypoints[0][15][1]),
                    int(keypoints[0][16][0]),
                    int(keypoints[0][16][1]),
                ]
                self.keypoint_pub.publish(int32_multi_array)

                # Visualize keypoints on the frame
                plotted_img = frame
                for i in range(len(keypoints[0])):
                    pt = keypoints[0][i]
                    ctr = (int(pt[0]), int(pt[1]))

                    # Draw circles at keypoints
                    plotted_img = cv2.circle(
                        plotted_img, ctr, radius=2, color=(0, 0, 255), thickness=-1
                    )

                    # Add keypoint index as text
                    plotted_img = cv2.putText(
                        plotted_img,
                        str(i),
                        ctr,
                        fontFace=0,
                        fontScale=1,
                        color=(0, 0, 255),
                        thickness=2,
                    )

                # Display the frame with keypoints
                cv2.imshow("YOLOv8 Inference", plotted_img)

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:  # Break the loop if the end of the video is reached
                break

        # Release the video capture object and close the display window
        self.cap.release()
        cv2.destroyAllWindows()

    def run_loop(self):
        """
        Call and print find keypoints
        """
        kpts = self.find_keypoints()
        print(kpts)


def main(args=None):
    rclpy.init(args=args)
    node = KeyPointsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
