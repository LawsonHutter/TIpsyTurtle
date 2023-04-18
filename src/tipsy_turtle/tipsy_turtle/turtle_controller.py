#!/usr/bin/env python3

# Packages and Libraries
import rclpy
import os
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from nav2_msgs.srv import LoadMap
from nav2_msgs.action import NavigateToPose
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String

# Other files

# Turtle Controller Node
class TurtleController(Node):
    # Inherit Node
    def __init__(self):
        # Unused Node Shell
        self.get_logger().info('Test')

# GPIO Publisher
class GPIO_Publisher_Node(Node):
    def __init__(self):
        super().__init__("gpio_pub")

        # publish
        self.gpio_publisher_ = self.create_publisher(String, "/rpi_gpio/msg", 10)

        # Sub and callback
        self.button_subscriber_ = self.create_subscription(String, "/rpi_button/msg", self.button_callback, 10)

        # Buttons
        self.buttons = {}

    def button_callback(self, msg: String):
        msg_sep = str(msg.data).split(":")
        print(msg_sep)
        self.buttons[str(msg_sep[0])] = msg_sep[1]

    def send_command(self, msg):
        self.gpio_publisher_.publish(msg)

    def poll_btn(self, name):        
        if self.buttons[name] == "True":
            return True
        else:
            return False

def send(message, node):
    msg = String()
    msg.data = message
    node.send_command(msg)

def main():

    # Initializations
    rclpy.init()
    node = GPIO_Publisher_Node()
    navigator = TurtleBot4Navigator()

    # Set up and zero servos
    send('SERVO:INIT:2:SERVO1', node)
    send('SERVO:MOVE:SERVO1:0', node)

    # Set up button
    send('BTN:INIT:3:BTN1', node)
    node.buttons["BTN1"] = "False"

    # Feedback
    print("Initializing")

    """
        Example Button code
        
        # Poll button to see if we should have another try
        while(1):
            while not node.poll_btn("BTN1"):
                print("Loop")
                send('BTN:CHECK:BTN1', node)
                rclpy.spin_once(node)

            print("Pressed")
    
    """

    """
        Example Servo code

        while (1):
            send('SERVO:MOVE:SERVO1:60', node)

            time.sleep(0.5)

            send('SERVO:MOVE:SERVO1:-15', node)

            time.sleep(0.5)
    """

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Feedback
    print("Waiting for Nav2")

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Feedback
    print("Starting Program")

    # Set goal points 
    goal_fridge = navigator.getPoseStamped([-5.301235675811768, 1.270430326461792], TurtleBot4Directions.EAST)
    goal_couch = navigator.getPoseStamped([-1.2631111145019531, 1.8153612613677979], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Main LOOP
    while(1):
        # Go to fridge
        navigator.startToPose(goal_fridge)

        # Devin, put your code here to get the beer

        # Go to couch
        navigator.startToPose(goal_couch)

        # Reset button (Potential bug here)
        rclpy.spin_once(node)
        node.buttons["BTN1"] = "False"

        # Poll button to see if we should have another try        
        while not node.poll_btn("BTN1"):
            send('BTN:CHECK:BTN1', node)

            time.sleep(1)

            rclpy.spin_once(node)

        time.sleep(5)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

header:
  stamp:
    sec: 1681694956
    nanosec: 140790603
  frame_id: map
point:  Fridge
  x: -5.301235675811768
  y: 1.270430326461792
  z: -0.005340576171875
---
header:
  stamp:
    sec: 1681694966
    nanosec: 418180486
  frame_id: map
point:  Bean bag
  x: -1.2631111145019531
  y: 1.8153612613677979
  z: -0.005340576171875
---

"""