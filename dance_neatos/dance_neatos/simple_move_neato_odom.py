import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math
from nav_msgs.msg import Odometry
from dance_neatos.helper_functions import convert, quaternion_to_euler

class MovementNodeOdom(Node):
    
    def __init__(self):
        super().__init__("move_neato_odom_node")
        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)

        self.neato_names = ['head', 'left_a', 'right_a', 'left_l', 'right_l'] # names of the neatos
        self.inits = [0, 0, 0, 0, 0, 0, 0] # initializing values that will eventually correspond to the below labels
        self.labels = ["initX", "initY", "currX", "currY", "currTheta", "targetX", "targetY"]
        
        self.neato_info = {
            "head" :  {
                'initX': 0.0,
                'initY': 0.0,
                'currX': 0.0,
                'currY': 0.0,
                'currTheta': 0.0,
                'targetX': 0.0,
                'targetY': 0.0,
                'keypoint_x' : 0,
                'keypoint_y' : 1
             },
            "left_a" :  {
                'initX': 0.0,
                'initY': 0.0,
                'currX': 0.0,
                'currY': 0.0,
                'currTheta': 0.0,
                'targetX': 0.0,
                'targetY': 0.0,
                'keypoint_x' : 2,
                'keypoint_y' : 3
             },
            "right_a" :  {
                'initX': 0.0,
                'initY': 0.0,
                'currX': 0.0,
                'currY': 0.0,
                'currTheta': 0.0,
                'targetX': 0.0,
                'targetY': 0.0,
                'keypoint_x' : 4,
                'keypoint_y' : 5
             },
            "left_l" :  {
                'initX': 0.0,
                'initY': 0.0,
                'currX': 0.0,
                'currY': 0.0,
                'currTheta': 0.0,
                'targetX': 0.0,
                'targetY': 0.0,
                'keypoint_x' : 6,
                'keypoint_y' : 7
             },
            "right_l" :  {
                'initX': 0.0,
                'initY': 0.0,
                'currX': 0.0,
                'currY': 0.0,
                'currTheta': 0.0,
                'targetX': 0.0,
                'targetY': 0.0,
                'keypoint_x' : 8,
                'keypoint_y' : 9
             }
         }

        self.head_pub = self.create_publisher(Twist, "head/cmd_vel", 10)
        self.left_a_pub = self.create_publisher(Twist, "left_a/cmd_vel", 10)
        self.right_a_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        self.left_l_pub = self.create_publisher(Twist, "left_l/cmd_vel", 10)
        self.right_l_pub = self.create_publisher(Twist, "right_l/cmd_vel", 10)

        self.create_subscription(Odometry, "head/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_l/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_l/odom", self.update_pose, 10)

    def process_keypoint(self, keypoint):       
        # for i in self.neato_names:
        for neato_name, neato_value in self.neato_info.items():
            print("Received keypoint", neato_name)
            
            # if the starting points of the neatos haven't been set to the positions of their
            # keypoints in the first frame, then do that

            if neato_value['initX']==0.0 and neato_value['initY']==0.0:
                print("init")
                initX, initY = convert(keypoint.data[neato_value['keypoint_x']], keypoint.data[neato_value['keypoint_y']])
                neato_value["initX"] = initX
                neato_value["initY"] = initY
            else:
                print("updating targets")
                targetX, targetY = convert(keypoint.data[neato_value['keypoint_x']], keypoint.data[neato_value['keypoint_y']])
                neato_value["targetX"] = targetX
                neato_value["targetY"] = targetY
                print(f"Target X: {targetX}, Target Y: {targetY}")
                self.move_neato(neato_name, neato_value)

    def update_pose(self, msg):
        neato_name = msg.header.frame_id[0:-4]

        # Extracting position (x, y) and putting them in dictionary
        self.neato_info[neato_name]["currX"] = msg.pose.pose.position.x
        self.neato_info[neato_name]["currY"] = msg.pose.pose.position.y

        # Extracting orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles. Put heading into dictionary
        _, _, self.neato_info[neato_name]["currTheta"] = quaternion_to_euler(quaternion)

    def move_neato(self, neato_name, neato_value):
        msg = Twist()

        # probably works, come back to if things don't
        publisher = getattr(self, f"{neato_name}_pub")
        
        # initialize variables so they can be used as the conditional for the while loop
        curr_x = neato_value["currX"]
        curr_y = neato_value["currY"]
        curr_theta = neato_value["currTheta"]
        new_x = neato_value["targetX"]
        new_y = neato_value["targetX"]
        print(f"current x: {curr_x}, y: {curr_y}, theta: {curr_theta}")

        delta_x = new_x - curr_x
        delta_y = new_y - curr_y
        vel = math.sqrt(delta_x**2 + delta_y**2)
        
        while delta_x > 0.03 or delta_y > 0.03:
            
            new_theta = math.atan2(new_y, new_x)
            delta_theta = new_theta - curr_theta
            # print(f"delta theta: {delta_theta}")

            msg.linear.x = vel
            msg.angular.z = delta_theta
            publisher.publish(msg)

            # recalculate for next round
            curr_x = neato_value["currX"]
            curr_y = neato_value["currY"]
            curr_theta = neato_value["currTheta"]
            new_x = neato_value["targetX"]
            new_y = neato_value["targetX"]
            print(f"AAAAAAAAAAAAAcurrent x: {curr_x}, y: {curr_y}, theta: {curr_theta}")

            delta_x = new_x - curr_x
            delta_y = new_y - curr_y
            vel = math.sqrt(delta_x**2 + delta_y**2)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNodeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()