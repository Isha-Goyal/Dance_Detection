import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math


class MovementNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.create_subscription(Int32MultiArray, 'keypoint', self.process_keypoint, 10)
        # self.timer = self.create_timer(.1, self.process_keypoint(keypoint))
        self.head_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.head_msg_pub = self.create_publisher(Marker, 'obstacle', 10)
        # starter values
        self.x_prev = 0.0
        self.y_prev = 0.0

    def process_keypoint(self, keypoint):
        msg = Twist()
        delta_x = keypoint.data[0] - self.x_prev
        delta_y = keypoint.data[1] - self.y_prev
        angular_turn = math.atan2(delta_y,delta_x)
        forward = math.sqrt(delta_x**2+delta_y**2)
        msg.angular.z = angular_turn
        msg.linear.x = forward
        self.head_vel_pub.publish(msg)

        # Set current keypoint to previous
        self.x_prev = keypoint.data[0]
        self.y_prev = keypoint.data[1]


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()