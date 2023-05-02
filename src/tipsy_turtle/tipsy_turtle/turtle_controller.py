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

import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import argparse
import numpy as np

# Other files

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

class Camera(Node):
	
    def __init__(self):
        super().__init__("person_following")
        self.subscription = self.create_subscription(Image, '/color/preview/image', self.couch_callback, 10)
        self.subscription
        self.br = CvBridge()
        
        # Can Grab Confirmation
        self.hasBeer = False
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)
                
    def can_init(self):
        self.subscription = self.create_subscription(Image, '/color/preview/image', self.can_callback, 10)

		
    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x
        self.cmd_vel_pub.publish(msg)
        #print(angular_z)
        #print(linear_x)


    def can_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
		
		#arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
		#arucoParams = cv2.aruco.DetectorParameters()
		
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
        arucoParams = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
		
		#detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
		#(corners, ids, rejected) = detector.detectMarkers(gray)
		
        if len(corners) > 0:
            #self.drive(0.5, 0.0)
            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                
                #(xA, yA, xB, yB) = corners
                #x_c, y_c = (xA + xB/2), (yA + yB/2)
                widthBounding = int(bottomRight[0] - topLeft[0])

                if widthBounding >= 58:
                    print("CAN GRABBED")
                    self.hasBeer = True
                else:
                    offset_x = cX - 125
                    theta = offset_x / 250
                    angular = - 2 * theta
                    self.drive(0.2, float(angular))

		
    def couch_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
                        
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        boxes, weights = hog.detectMultiScale(gray, winStride=(8,8))
        boxes = np.array([[x, y, x+w, y+h] for (x, y, w, h) in boxes])
        for (xA, yA, xB, yB) in boxes:
            self.get_logger().info('Detecting Person')
            x_c, y_c = (xA + xB/2), (yA + yB/2)
            offset_x = x_c - 125
            theta = offset_x / 250
            angular = - 2 * theta
            self.drive(0.5, angular)

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

def lift(node):
    send('MODE:LOW:7', node)
    send('MODE:HIGH:8', node)
    time.sleep(3)

def lower(node):
    send('MODE:HIGH:7', node)
    send('MODE:LOW:8', node)
    time.sleep(3.5)


def main():

    # Initializations
    rclpy.init(args=args)
    node = GPIO_Publisher_Node()
    camNode = Camera()
    navigator = TurtleBot4Navigator()

    # Set up and zero servos
    send('INIT:OUT:7', node) # UP
    send('INIT:OUT:8', node) # DOWN
    send('MODE:LOW:7', node)
    send('MODE:LOW:8', node)

    send('SERVO:INIT:2:SERVO1', node)
    send('SERVO:MOVE:SERVO1:0', node)

    send('SERVO:INIT:21:SERVO21', node)
    send('SERVO:MOVE:SERVO21:0', node)

    time.sleep(1)

    # Set up button
    send('BTN:INIT:3:BTN1', node)
    node.buttons["BTN1"] = "False"

    print("BEGIN")

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

    """

    # Feedback
    print("Starting Program")

    # Set goal points 
    #goal_fridge = navigator.getPoseStamped([-5.301235675811768, 1.270430326461792], TurtleBot4Directions.EAST)
    #goal_couch = navigator.getPoseStamped([-1.2631111145019531, 1.8153612613677979], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Set initial point of frdge
    while(1):
        # Reset button (Potential bug here)
        rclpy.spin_once(node)
        node.buttons["BTN1"] = "False"

        # Poll button to see if we should have another try        
        while not node.poll_btn("BTN1"):
            send('BTN:CHECK:BTN1', node)

            time.sleep(1)

            rclpy.spin_once(node)

        # Once button pressed, get fridge location
        goal_fridge = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        navigator.setInitialPose(goal_fridge)
        print(goal_fridge)

        # Wait for Nav2
        print("Waiting for Nav2")
        navigator.waitUntilNav2Active()

        break

    # Set initial point of couch
    while(1):

        rclpy.spin_once(camNode)

        # Reset button (Potential bug here)
        rclpy.spin_once(node)
        node.buttons["BTN1"] = "False"

        # Poll button to see if we should have another try        
        while not node.poll_btn("BTN1"):
            send('BTN:CHECK:BTN1', node)

            time.sleep(1)

            rclpy.spin_once(node)

        # Set couch pos
        goal_couch = navigator.stampPose()

        # Set to track Can
        camNode.can_init()

        # Feedback
        print(goal_couch)
        break
            

    # Feedback message
    print("Starting Main Loop")

    # Main LOOP
    while(1):
        # Go to fridge
        navigator.startToPose(goal_fridge)

        # Grab Can
        while(1):
            print("Looking for beer")

            # Run code once
            rclpy.spin_once(camNode)

            if camNode.hasBeer:
                print("Have Beer")
                camNode.hasBeer = False
                break

        # Lift Can
        lift(node)

        # Go to couch
        navigator.startToPose(goal_couch)

        # Reset button (Potential bug here)
        rclpy.spin_once(node)
        node.buttons["BTN2"] = "False"

        # Poll button to see if we should have another try        
        while not node.poll_btn("BTN2"):
            send('BTN:CHECK:BTN2', node)

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

