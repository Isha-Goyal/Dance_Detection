import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math
from nav_msgs.msg import Odometry

class Neato:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.cmd_vel_pub = None
        self.odom_sub = None

    def initialize_publishers(self, node):
        self.cmd_vel_pub = node.create_publisher(Twist, f"{self.robot_name}/cmd_vel", 10)
        
        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_vel_pub.publish(msg)

    def initialize_subscribers(self, node):
        self.odom_sub = node.create_subscription(Odometry, f"{self.robot_name}/odom", self.odom_callback, 10)

    def odom_callback(self, msg):
        # Process Odometry data here
        print("")

class MovementNodeOdom(Node):
    
    def __init__(self):
        super().__init__("move_neato_odom_node")
        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)

        self.robots = []

        # Create and initialize robots
        self.create_robot("right_l")
        # self.create_robot("robot2")



    def process_keypoint(self):
        pass

        # self.create_subscription(Odometry, "head/odom", self.update_pose, 10)
        # self.create_subscription(Odometry, "left_a/odom", self.update_pose, 10)
        # self.create_subscription(Odometry, "right_a/odom", self.update_pose, 10)
        # self.create_subscription(Odometry, "left_l/odom", self.update_pose, 10)
        # self.create_subscription(Odometry, "right_l/odom", self.update_pose, 10)

        # self.head_vel_pub = self.create_publisher(Twist, "head/cmd_vel", 10)
        # self.left_arm_pub = self.create_publisher(Twist, "left_a/cmd_vel", 10)
        # self.right_arm_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        # self.left_leg_pub = self.create_publisher(Twist, "left_l/cmd_vel", 10)
        # self.right_leg_pub = self.create_publisher(Twist, "right_l/cmd_vel", 10)


    def create_robot(self, robot_name):
        robot = Neato(robot_name)
        robot.robot_name
        robot.initialize_publishers(self)
        robot.initialize_subscribers(self)
        self.robots.append(robot)
    

def main(args=None):
    rclpy.init(args=args)
    node = MovementNodeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

