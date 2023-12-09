import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
import math


class MovementNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoider_node")
        self.right_arm_pub = self.create_publisher(Twist, "right_a/cmd_vel", 10)
        
        self.create_subscription(Odometry, "odom", self.update_pose, 10)
        print('subscription created')

    def update_pose(self, msg):

        print('update pose')

        # Extracting position (x, y)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extracting orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles
        _, _, heading = self.quaternion_to_euler(quaternion)

        print(f"Odometry Information - X: {self.x}, Y: {self.y}, Heading: {heading * (180 / 3.14159265359)} degrees")

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
        t1 = +1.0 - 2.0 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1])
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        t4 = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()