#!/usr/bin/env python3
import csv
from math import sqrt, pow
import sys
import threading
import time

import rospy
import numpy as np
import os

# Our custom messages
from robo_forger.msg import Drawing
from robo_forger.msg import Point

# Import our geometry and sensor messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# How far our robot should be from the wall
WALL_DISTANCE = .1  # m
# TODO: Find measurement
# The arms offset from the edge of the robot
ARM_DISTANCE_OFFSET = 0  # m
# How long our robot's arm is
ARM_LENGTH = 0  # m


# Returns a canvas size our robot uses to determine where to draw
def max_height():
    # This is the highest the robot can draw given our configuration dimensions
    ret = sqrt(pow(ARM_LENGTH, 2) - pow(WALL_DISTANCE + ARM_DISTANCE_OFFSET, 2))
    print("Robot can draw ", ret, " m high")
    return ret

'''
This class implements implements our RoboForger functionality

High Level Description:

Our controller:
- Reads an image in as input
- Converts that image into a series of points 
- Constrains those points to the robot's dimensions
- Aligns the robot:
    - So that its left side is parallel with a wall at a specifies WALL_DISTANCE
    - So that it's arm is at 90 degrees in the x-direction relative to it's origin
    - and it's gripper is holding a marker at the real-world height of it's origin, touching the wall
- This places the marker at a well-defined origin for our drawing
- Draws the image:
    - For each point
        - If the point is continuous
            - move the marker along the wall to the point
        - otherwise
            - pickup and place the marker at the point
    - Movement is achieved by 
        - translating the y-coord of the marker using reverse kinematics to raise and lower the arm
        - and the x-coord by using odometry to move the robot along the wall
'''


class RoboForger(object):
    def __init__(self):
        rospy.init_node("robo_forger")

        # Declare our ROS objects
        self.initialized = False

        # A Drawing subscriber to accept drawings from our image reader node
        rospy.Subscriber("/robo_forger/drawing", Drawing, self.drawing_callback)

        # Odometry subscriber

        # Velocity Publisher

        # Whatever we need for the arms

        self.align()

        self.initialized = True

    def drawing_callback(self, drawing):
        print("[ROBO-FORGER] Received new drawing")
        if not self.initialized:
            print("[ROBO-FORGER] Not initialized! Refusing drawing.")
            return

        # For now initialize a default drawing!
        drawing = Drawing()
        drawing.points = [
            Point(x=0, y=0, continuous=True),
            Point(x=0, y=5, continuous=True),
            Point(x=5, y=5, continuous=True),
            Point(x=5, y=0, continuous=True)
        ]
        drawing.height = 6
        drawing.width = 6
        self.draw(drawing)

    # Align our robot with the wall and its arm with our origin
    def align(self):
        # TODO: Implement approaching the wall and turning
        pass

    # Recognize image and load it into our points array
    def draw(self, drawing):
        print("[ROBO-FORGER] Drawing ", drawing.width, "X", drawing.height, " image")

        # The drawing height and width are in pixels, which we need to convert to m
        scalar = float(max_height() / drawing.height)  # Resolution of the largest m / pixel value we can use

        print("[ROBO-FORGER] Using drawing scalar: ", scalar)

        for point in drawing.points:
            # Move the marker across the wall
            self.move_marker(
                x=point.x * scalar,
                y=point.y * scalar,
                continuous=point.continuous
            )
        print("[ROBO-FORGER] Done drawing")

    # Control how we move the marker
    def move_marker(self, x, y, continuous):
        print("Moving marker", x, "X", y, " Continuously" if continuous else " Lifting")
        # TODO: move the marker
        pass

    def run(self):
        print("[ROBO-FORGER] Waiting for controller to be initialized")
        while not self.initialized:
            print("...")
            time.sleep(1)
        print("[ROBO-FORGER] Running.")
        rospy.spin()


if __name__ == "__main__":
    node = RoboForger()
    node.run()
