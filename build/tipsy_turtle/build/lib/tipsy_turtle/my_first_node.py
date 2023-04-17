#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    # Inherit Node with super()
    def __init__(self):
        super().__init__("first_node")

        # Member varable
        self.counter_ = 0

        # Format to send cl messages
        self.get_logger().info("Hello World")

        self.create_timer(1.0, self.timer_callback)

    # Example Callback (wont run without spin)
    def timer_callback(self):
        self.get_logger().info("Hello Callback " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    # First line in main
    rclpy.init(args=args)
    
    # Put code here
    node = MyNode()

    # Runs node forever until killed by ctrl C
    # Allows for callbacks to run
    rclpy.spin(node)

    # Last line in main
    rclpy.shutdown()


