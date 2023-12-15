"""
Moves up to 5 Neatos in the X Y direction.
"""
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
            },
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
                neato_value["targetX"] = targetX - neato_value["initX"]
                neato_value["targetY"] = targetY - neato_value["initY"]

                # Move the Neato robot to the new target position
                self.move_neato(neato_value)

    def update_pose(self, msg: Odometry):
        """
        Update the pose information for a Neato robot based on a received ROS message.

        Args:
            msg (Odometry): A ROS message containing pose information.
        """
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
        # Initialize a Twist message to control the Neato's movement
        msg = Twist()

        # Retrieve the ROS publisher for the Neato robot
        publisher = neato_value["publisher"]

        # Calculate the change in position along the X and Y axes
        delta_x = neato_value["targetX"] - neato_value["currX"]
        delta_y = neato_value["targetY"] - neato_value["currY"]

        # Check if the Neato needs to move significantly
        if abs(delta_x) > 0.1 or abs(delta_y) > 0.1:
            # Calculate the new heading angle towards the target position
            new_theta = math.atan2(delta_y, delta_x)
            delta_theta = new_theta - neato_value["currTheta"]

            # Set the linear velocity based on the change in X position
            msg.linear.x = abs(delta_x)

            # Check if the Neato needs to rotate to align with the target heading
            if abs(delta_theta) > 0.2:
                msg.angular.z = abs(delta_theta)
            else:
                msg.angular.z = 0.0

            # Publish the movement commands
            publisher.publish(msg)

        # If the Neato is close to the target, stop its movement
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
