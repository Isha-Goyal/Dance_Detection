import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math
from nav_msgs.msg import Odometry
from dance_neatos.helper_functions import convert, quaternion_to_euler

class Neato:
    def __init__(self, name, kypt_x_index, kypt_y_index):
        self.name = name
        self.cmd_vel_pub = None
        self.odom_sub = None
        self.x_index = kypt_x_index # index of the x and y for this body part in the keypoint matrix
        self.y_index = kypt_y_index
        self.init_x = 0 # in meters, the starting location of the neato relative to map frame
        self.init_y = 0
        self.x = 0
        self.y = 0
        self.heading = 0

    def initialize_publishers(self, node):
        self.cmd_vel_pub = node.create_publisher(Twist, f"{self.name}/cmd_vel", 10)

    def initialize_subscribers(self, node):
        self.odom_sub = node.create_subscription(Odometry, f"{self.name}/odom", self.odom_callback, 10)

    def odom_callback(self, msg):

        # Extracting position (x, y)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        print(f"odom --> x: {self.x}, y: {self.y}")

        # Extracting orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles
        _, _, self.heading = quaternion_to_euler(quaternion)

        # print(f"Odometry Information - X: {self.x}, Y: {self.y}, Heading: {self.heading * (180 / 3.14159265359)} degrees")

    def set_init(self, x, y):
        self.init_x = x
        self.init_y = y

    def publish_msg(self, lin_vel, ang_vel):
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.cmd_vel_pub.publish(msg)
    
class MovementNodeOdom(Node):
    
    def __init__(self):
        super().__init__("move_neato_odom_node")
        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)

        self.robots = []

        # Create and initialize robots
        self.create_robot("right_l", 4, 5)
        self.neato= self.robots[0]
        # print('created')
        # self.create_robot("robot2")

        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)


    def process_keypoint(self, keypoint):
        
        # initialize positions based on first frame
        if self.neato.init_x == 0 and self.neato.init_y == 0:
            x, y = convert(keypoint.data[self.neato.x_index], keypoint.data[self.neato.y_index])
            self.neato.set_init(x, y)
            # print(f"init: {x, y}")

        # current position of robot in meters wrt map frame
        # convert to odom frame by subtracting initial position
        new_x, new_y = convert(
            keypoint.data[self.neato.x_index] - self.neato.init_x, 
            keypoint.data[self.neato.y_index] - self.neato.init_y)
        # print(f"new coord: {new_x, new_y}")
        
        self.move_neato(self.neato, new_x, new_y)

    def move_neato(self, robot, new_x, new_y):

        # print("test move")
        # self.neato.publish_msg(0, 0.5)

        curr_x = robot.x
        curr_y = robot.y
        curr_theta = robot.heading
        print(f"current x: {curr_x}, y: {curr_y}")

        delta_x = new_x - curr_x
        delta_y = new_y - curr_y
        vel = math.sqrt(delta_x**2 + delta_y**2)
        # print(f"delta x: {delta_x}, y: {delta_y}")

        while delta_x > 0.03 or delta_y > 0.03:
            new_theta = math.atan2(new_y, new_x)
            delta_theta = new_theta - curr_theta
            # print(f"delta theta: {delta_theta}")

            self.neato.publish_msg(vel, delta_theta)

            # print("msgs set")

            curr_x = robot.x
            curr_y = robot.y
            curr_theta = robot.heading
            # print(f"current x: {curr_x}, y: {curr_y}")

            delta_x = new_x - curr_x
            delta_y = new_y - curr_y
            vel = math.sqrt(delta_x**2 + delta_y**2)
            # print(f"delta x: {delta_x}, y: {delta_y}")
        
        

    def create_robot(self, name, ind_x, ind_y):
        robot = Neato(name, ind_x, ind_y)
        robot.name
        robot.initialize_publishers(self)
        robot.initialize_subscribers(self)
        self.robots.append(robot)
        return robot
    

def main(args=None):
    rclpy.init(args=args)
    node = MovementNodeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

