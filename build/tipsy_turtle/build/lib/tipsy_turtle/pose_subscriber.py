#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped

# This mode draws a circle in tutlesim
class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_sub")

        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(str(msg))

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()