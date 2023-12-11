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
        self.dict = {}

        for i in self.neato_names:
            self.dict.update({i:self.inits})

        self.head_vel_pub = self.create_publisher(Twist, "head/cmd_vel", 10)
        self.left_arm_pub = self.create_publisher(Twist, "left_a/cmd_vel", 10)
        self.right_arm_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        self.left_leg_pub = self.create_publisher(Twist, "left_l/cmd_vel", 10)
        self.right_leg_pub = self.create_publisher(Twist, "right_l/cmd_vel", 10)

        self.create_subscription(Odometry, "head/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_l/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_l/odom", self.update_pose, 10)

    def process_keypoint(self, keypoint):

        
        for i in self.neato_names:
            # set indices for each neato from the keypoints array
            x = self.neato_names.index(i)
            y = x + 1
            
            # if the starting points of the neatos haven't been set to the positions of their
            # keypoints in the first frame, then do that
            if dict[i][0]==0:
                dict[i][self.labels.index("initX")] = convert(keypoint.data[x]) # distance
                dict[i][self.labels.index("initY")] = convert(keypoint.data[y]) # distance
            else:
                dict[i][self.labels.index("targetX")] = convert(keypoint.data[x]) # distance
                dict[i][self.labels.index("targetY")] = convert(keypoint.data[y]) # distance

        # for i in range(len(self.neatos)):
        #     self.move_neato()

    def update_pose(self, msg):
        neato = msg.header.frame_id[0:-5]

        # Extracting position (x, y) and putting them in dictionary
        self.dict[neato][self.labels.index("currX")]= msg.pose.pose.position.x
        self.dict[neato][self.labels.index("currY")] = msg.pose.pose.position.y
        print(f"odom --> x: {self.x}, y: {self.y}")

        # Extracting orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles
        _, _, dict[neato][self.labels.index("currTheta")] = quaternion_to_euler(quaternion)
        

def main(args=None):
    rclpy.init(args=args)
    node = MovementNodeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()