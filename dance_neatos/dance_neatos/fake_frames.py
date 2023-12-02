import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math
import random

class FramesNode(Node):
    def __init__(self):
        super().__init__('frames_node')
        self.timer = self.create_timer(.1, self.run_loop)
        self.frames = self.create_publisher(Int32MultiArray, 'keypoint', 10)
        self.x_pos = 0.0
        self.y_pos = 0.0

    def run_loop(self):
        self.x_pos += random.random()
        self.y_pos += random.uniform(-1,1)
        self.frames.publish([self.x_pos,self.y_pos])


def main(args=None):
    rclpy.init(args=args)
    node = FramesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()