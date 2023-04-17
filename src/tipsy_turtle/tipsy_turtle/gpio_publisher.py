#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class GPIO_Publisher_Node(Node):
    def __init__(self):
        super().__init__("gpio_pub")

        # publish
        self.gpio_publisher_ = self.create_publisher(String, "/rpi_gpio/msg", 10)

    def send_command(self, msg):
        self.gpio_publisher_.publish(msg)
