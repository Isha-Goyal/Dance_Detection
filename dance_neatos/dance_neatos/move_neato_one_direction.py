"""
Moves up to 5 Neatos in the X direction.
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

    def process_keypoint(self, keypoint: Int32MultiArray):
        """
        Process a received keypoint message, updating Neato robot positions.

        This method takes a keypoint message and updates the positions of Neato robots
        based on the keypoint information. The keypoint data is used to compute the initial
        and target positions of each Neato robot. If the initial positions haven't been set,
        they are initialized with the first received keypoint. Subsequent keypoints are used
        to update the target positions, and the `move_neato` method is called to perform
        the necessary actions.

        Args:
            keypoint (Int32MultiArray): A ROS Int32MultiArray containing keypoint information.
        """
        for neato_name, neato_value in self.neato_info.items():
            # Check if the initial positions of the neatos haven't been set
            if neato_value["initX"] == 0.0 and neato_value["initY"] == 0.0:
                # Set initial positions to the positions of their keypoints in the first frame
                initX, initY = convert(
                    keypoint.data[neato_value["keypoint_x"]],
                    keypoint.data[neato_value["keypoint_y"]],
                )
                neato_value["initX"] = initX
                neato_value["initY"] = initY
            else:
                # Compute target positions from the current keypoint
                targetX, targetY = convert(
                    keypoint.data[neato_value["keypoint_x"]],
                    keypoint.data[neato_value["keypoint_y"]],
                )
                neato_value["targetX"] = targetX
                neato_value["targetY"] = targetY

                # Move the Neato robot to the new target position
                self.move_neato(neato_value)

    def update_pose(self, msg: Odometry):
        """
        Update the pose information for a Neato robot based on a received ROS message.

        Args:
            msg (Odometry): A ROS message containing pose information.
        """
        # Extract Neato name from the ROS message header
        neato_name = msg.header.frame_id[0:-4]

        # Extract and update current position (x, y) in the neato_info dictionary
        self.neato_info[neato_name]["currX"] = msg.pose.pose.position.x
        self.neato_info[neato_name]["currY"] = msg.pose.pose.position.y

        # Extract quaternion from the ROS message orientation
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        # Convert quaternion to Euler angles and update the current heading (theta)
        _, _, self.neato_info[neato_name]["currTheta"] = quaternion_to_euler(quaternion)

    def move_neato(self, neato_value: dict):
        """
        Move a Neato robot to a target position in the X axis.

        This method calculates the required linear velocity to move a Neato robot from its current
        position to a target position specified in the `neato_value` dictionary.

        Args:
            neato_value (dict): Dictionary containing information about the Neato robot, including
                current position (`currX`, `currY`), current heading (`currTheta`), target position
                (`targetX`, `targetY`), and ROS publisher (`publisher`).
        """
        msg = Twist()

        # Retrieve the ROS publisher for the Neato robot
        publisher = neato_value["publisher"]

        # Calculate the change in x and y coordinates
        delta_x = neato_value["targetX"] - neato_value["currX"]

        # Check if movement is required (based on threshold)
        if abs(delta_x) > 0.03:
            msg.linear.x = delta_x
            publisher.publish(msg)
        else:
            # Stop movement if the Neato robot is close to the target position
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
