import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math
import random
import numpy as np

class FramesNode(Node):
    def __init__(self):
        super().__init__('frames_node')
        self.timer = self.create_timer(.1, self.run_loop)
        self.frames = self.create_publisher(Int32MultiArray, 'keypoint', 10)
        self.x_pos = 0.0
        self.y_pos = 0.0

    def run_loop(self):
        self.x_pos += random.random()
        self.y_pos += random.random()

        # Convert float coordinates to integers
        x_int = int(self.x_pos)
        y_int = int(self.y_pos)

        int32_multi_array = Int32MultiArray()
        int32_multi_array.data = [x_int, y_int]
        self.frames.publish(int32_multi_array)


def main(args=None):
    rclpy.init(args=args)
    node = FramesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()