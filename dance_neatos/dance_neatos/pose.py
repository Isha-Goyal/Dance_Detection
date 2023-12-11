import numpy as np


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
        
    def create_pose(self, v_pose):
        """
            Create a pose matrix of the particles position.
            v_pose: is a tuple of the x, y, and theta of a particle

            returns: an array of the pose matrix
        """
        pose_matrix = np.array([
            [np.cos(v_pose[2]), -np.sin(v_pose[2]), v_pose[0]],
            [np.sin(v_pose[2]), np.cos(v_pose[2]), v_pose[1]],
            [0,0,1]
        ])
        return pose_matrix

    new_odom_matrix = self.create_pose(new_odom_xy_theta)
    curr_odom_matrix = self.create_pose(self.current_odom_xy_theta)
    delta_matrix = np.linalg.inv(curr_odom_matrix) @ new_odom_matrix