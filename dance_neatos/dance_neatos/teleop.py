import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.timer = self.create_timer(0.1, self.run_loop)
        self.paul_vel_pub = self.create_publisher(Twist, 'paul/cmd_vel', 10)
        self.alice_vel_pub = self.create_publisher(Twist, 'alice/cmd_vel', 10)
        self.isha_vel_pub = self.create_publisher(Twist, 'isha/cmd_vel', 10)

    def run_loop(self):
        # runs 10 times every second
        key = self.getKey()
        msg = Twist()
        # switch statement to set velocity based on key pressed
        if key == 'w':
            msg.linear.x = 0.5
        elif key == 's':
            msg.linear.x = -0.5
        elif key == 'a':
            msg.angular.z = 1.0
        elif key == 'd':
            msg.angular.z = -1.0
        elif key == 'q':
            msg.linear.x = 0.3
            msg.angular.z = 0.5
        elif key == 'e':
            msg.linear.x = 0.3
            msg.angular.z = -0.5
        elif key == 'z':
            msg.linear.x = -0.3
            msg.angular.z = 0.5
        elif key == 'x':
            msg.linear.x = -0.3
            msg.angular.z = -0.5
        elif key == '\x03':
            self.destroy_node()
        # publish the velocity
        self.paul_vel_pub.publish(msg)
        self.alice_vel_pub.publish(msg)
        self.isha_vel_pub.publish(msg)

    def getKey(self): 
        # tracks which key is pressed on the keyboard
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
