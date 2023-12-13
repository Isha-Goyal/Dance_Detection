"""
Moves up to 5 neatos in the X direction.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from dance_neatos.helper_functions import convert, quaternion_to_euler


class MovementNodeOdom(Node):
    def __init__(self):
        super().__init__("move_neato_odom_node")
        self.create_subscription(Int32MultiArray, "keypoint", self.process_keypoint, 10)

        # Create publishers to all five neatos
        self.head_pub = self.create_publisher(Twist, "head/cmd_vel", 10)
        self.left_a_pub = self.create_publisher(Twist, "left_a/cmd_vel", 10)
        self.right_a_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        self.left_l_pub = self.create_publisher(Twist, "left_l/cmd_vel", 10)
        self.right_l_pub = self.create_publisher(Twist, "right_l/cmd_vel", 10)

        # A dictionary to store all information for each neato
        self.neato_info = {
            "head": {
                "initX": 0.0,
                "initY": 0.0,
                "currX": 0.0,
                "currY": 0.0,
                "currTheta": 0.0,
                "targetX": 0.0,
                "targetY": 0.0,
                "keypoint_x": 0,
                "keypoint_y": 1,
                "publisher": self.head_pub,
            },
            "left_a": {
                "initX": 0.0,
                "initY": 0.0,
                "currX": 0.0,
                "currY": 0.0,
                "currTheta": 0.0,
                "targetX": 0.0,
                "targetY": 0.0,
                "keypoint_x": 2,
                "keypoint_y": 3,
                "publisher": self.left_a_pub,
            },
            "right_a": {
                "initX": 0.0,
                "initY": 0.0,
                "currX": 0.0,
                "currY": 0.0,
                "currTheta": 0.0,
                "targetX": 0.0,
                "targetY": 0.0,
                "keypoint_x": 4,
                "keypoint_y": 5,
                "publisher": self.right_a_pub,
            },
            "left_l": {
                "initX": 0.0,
                "initY": 0.0,
                "currX": 0.0,
                "currY": 0.0,
                "currTheta": 0.0,
                "targetX": 0.0,
                "targetY": 0.0,
                "keypoint_x": 6,
                "keypoint_y": 7,
                "publisher": self.left_l_pub,
            },
            "right_l": {
                "initX": 0.0,
                "initY": 0.0,
                "currX": 0.0,
                "currY": 0.0,
                "currTheta": 0.0,
                "targetX": 0.0,
                "targetY": 0.0,
                "keypoint_x": 8,
                "keypoint_y": 9,
                "publisher": self.right_l_pub,
            },
        }

        # Create subscribers to get the odometry of all five neatos
        self.create_subscription(Odometry, "head/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_a/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "left_l/odom", self.update_pose, 10)
        self.create_subscription(Odometry, "right_l/odom", self.update_pose, 10)

    def process_keypoint(self, keypoint):
        for neato_name, neato_value in self.neato_info.items():

            # if the starting points of the neatos haven't been set to the positions of their
            # keypoints in the first frame, then do that

            if neato_value["initX"] == 0.0 and neato_value["initY"] == 0.0:
                initX, initY = convert(
                    keypoint.data[neato_value["keypoint_x"]],
                    keypoint.data[neato_value["keypoint_y"]],
                )
                neato_value["initX"] = initX
                neato_value["initY"] = initY
                # print(f"init y: {initY}")
            else:
                # print(f"keypoint: {keypoint.data[neato_value['keypoint_y']]}")
                targetX, targetY = convert(
                    keypoint.data[neato_value["keypoint_x"]],
                    keypoint.data[neato_value["keypoint_y"]],
                )
                neato_value["targetX"] = targetX
                neato_value["targetY"] = targetY
                # print(f"Target Y: {targetY}")
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
            msg.pose.pose.orientation.w,
        )

        # Convert quaternion to Euler angles. Put heading into dictionary
        _, _, self.neato_info[neato_name]["currTheta"] = quaternion_to_euler(quaternion)

    def move_neato(self, neato_name, neato_value):
        msg = Twist()

        # probably works, come back to if things don't
        # publisher = getattr(self, f"{neato_name}_pub")
        publisher = neato_value["publisher"]

        # initialize variables so they can be used as the conditional for the while loop
        curr_x = neato_value["currX"]
        curr_y = neato_value["currY"]
        curr_theta = neato_value["currTheta"]
        new_x = neato_value["targetX"]
        new_y = neato_value["targetY"]
        print(f"x: {curr_x}")
        print(f"target: {new_x}")

        delta_x = new_x - curr_x
        # delta_y = new_y - curr_y
        # vel = math.sqrt(delta_x**2 + delta_y**2)
        # print(f"delta y: {delta_y}")

        if abs(delta_x) > 0.03:

            # new_theta = math.atan2(new_y, new_x)
            # delta_theta = new_theta - curr_theta
            # # print(f"delta theta: {delta_theta}")

            msg.linear.x = delta_x
            # msg.angular.z = delta_theta
            publisher.publish(msg)

        else:
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