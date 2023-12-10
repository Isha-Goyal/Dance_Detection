import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math


class MovementNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoider_node")
        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)
        self.head_vel_pub = self.create_publisher(Twist, "head/cmd_vel", 10)
        self.left_arm_pub = self.create_publisher(Twist, "left_a/cmd_vel", 10)
        self.right_arm_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        self.left_leg_pub = self.create_publisher(Twist, "left_l/cmd_vel", 10)
        self.right_leg_pub = self.create_publisher(Twist, "right_l/cmd_vel", 10)
        # starter values
        self.coord_prev = [0.0]*10

    def process_keypoint(self, keypoint):
        # msg = Twist()
        # delta_x = keypoint.data[0] - self.x_prev
        # delta_y = keypoint.data[1] - self.y_prev
        # angular_turn = math.atan2(delta_y, delta_x)
        # forward = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # msg.angular.z = angular_turn
        # msg.linear.x = forward
        # self.head_vel_pub.publish(msg)
        self.move_neato(keypoint, 0, 1,self.head_vel_pub)
        self.move_neato(keypoint, 2, 3,self.left_arm_pub)
        self.move_neato(keypoint, 4, 5,self.right_arm_pub)
        self.move_neato(keypoint, 6, 7,self.left_leg_pub)
        self.move_neato(keypoint, 8, 9,self.right_leg_pub)
    
    def move_neato(self, keypoint, x, y, neato):
        msg = Twist()
        delta_x = keypoint.data[x] - self.coord_prev[x]
        delta_y = keypoint.data[y] - self.coord_prev[y]

        angle = math.atan2(delta_y, delta_x)
        # angle_sign = math.copysign(1, angle)
        # move_sign = 1
        # if abs(angle) > math.pi/2:
        #     move_sign = -1
        #     angle = angle - math.pi/2
        
        forward = math.sqrt(delta_x ** 2 + delta_y ** 2)
        msg.angular.z = angle #* angle_sign
        msg.linear.x = forward *.1 #* move_sign
        neato.publish(msg)

        # Set current keypoint to previous
        self.coord_prev[x] = keypoint.data[x]
        self.coord_prev[y] = keypoint.data[y]


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
