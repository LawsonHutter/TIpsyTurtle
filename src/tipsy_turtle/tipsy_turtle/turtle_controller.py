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

# Other files
import gpio_publisher


# Turtle Controller Node
class TurtleController(Node):
    # Inherit Node
    def __init__(self):
        # Unused Node Shell
        self.get_logger().info('Test')

def main():

    # Initializations
    rclpy.init()
    node = gpio_publisher()

    frequency = 50 # 50Hz
    period = 1/frequency # Period in seconds
    half_period = period/2 # Half period in seconds

    while (1):
        node.send_command("2:HIGH")
        time.sleep(half_period)
        node.send_command("2:LOW")
        time.sleep(half_period)

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal points
    goal_fridge = navigator.getPoseStamped([-5.301235675811768, 1.270430326461792], TurtleBot4Directions.EAST)
    goal_couch = navigator.getPoseStamped([-1.2631111145019531, 1.8153612613677979], TurtleBot4Directions.EAST)


    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_fridge)

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