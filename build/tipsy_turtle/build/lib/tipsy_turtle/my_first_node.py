#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    # Inherit Node with super()
    def __init__(self):
        super().__init__("first_node")

        # Format to send cl messages
        self.get_logger().info("Hello World")




def main(args=None):
    # First line in main
    rclpy.init(args=args)

    # Put code here
    node = MyNode()

    # Runs node forever until killed by ctrl C
    rclpy.spin(node)

    # Last line in main
    rclpy.shutdown()


if __name__ == '__main__':
    main()
