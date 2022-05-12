#!/usr/bin/env python3
import csv
from math import sqrt, pow, inf
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


# Get the distance between a start position and an end position
def get_distance(start_pos, end_pos):
    # print("Start Pos: ", startPos, " | End pos: ", endPos)
    (start_x, start_y) = start_pos # Gotta love tuple unpacking
    (end_x, end_y) = end_pos
    ret = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    # print("Robot has moved", ret, "m")
    return ret


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

        # Class state variables
        self.class_initialized = False
        self.linear_speed = .05  # m/s
        self.angular_speed = .1  # rad/s
        self.wall_distance = WALL_DISTANCE + ARM_DISTANCE_OFFSET # m

        # Variables to hold
        self.odom_pose = None  # our current odometry state
        self.scan_ranges = None  # our current scan state

        # Variables that access specific scan ranges
        self.scan_max_range = None  # Our maximum scan range
        self.scan_front_dist = None  # How far the object directly in front of us is
        self.scan_front_bumper = None  # A scan range for our front bumper (for keeping the robot aligned)
        self.scan_rear_bumper = None  # A scan range for our rear bumper (for keeping the robot aligned)

        # Declare our ROS objects

        # Rospy Params
        self.queue_size = 20  # The size of our message queues
        self.rateLimit = rospy.Rate(10)  # How often we publish messages (2 Hz), utilize with self.rateLimit.sleep()

        # A Drawing subscriber to accept drawings from our image reader node
        rospy.Subscriber("/robo_forger/drawing", Drawing, self.drawing_callback)

        # Odometry subscriber
        rospy.Subscriber('/odom', Odometry, self.set_odom_pose)

        # Scan subscriber
        rospy.Subscriber('/scan', LaserScan, self.set_scan_data)

        # Velocity Publisher
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=self.queue_size)


        # Whatever we need for the arms

        self.align()

        self.class_initialized = True

    def initialized(self):
        return self.initialized \
            and self.scan_data is not None \
            and self.odom_pose is not None

    def set_scan_data(self, msg):
        # Filter our ranges for any bad readings and set them to the max range
        ranges = list(map(lambda x: x if x > 0 else msg.range_max, msg.ranges))
        self.scan_max_range = msg.range_max
        self.scan_ranges = ranges

        # Get the distance right in front of us
        self.scan_front_dist = ranges[0]

        # Our bumpers for wall following are on the left side of the robot

        # Get the distance of our front bumper
        self.scan_front_bumper = ranges[45]

        # Get the distance of rear bumper
        self.scan_rear_bumper = ranges[135]


    def set_odom_pose(self, msg):
        # Extract the pose from the odometry message
        # Defintion: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        position = msg.pose.pose.position
        self.odom_pose = (position.x, position.y)

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
    # Assumes the robot is at least kinda facing a wall
    def align(self):
        print("[ROBO-FORGER] Aligning robot with nearest wall in sight...")
        movement = Twist()

        print("[ROBO-FORGER] Approaching a wall...")
        # While We're not close to a wall to draw on
        while self.wall_distance < self.scan_front_dist:
            # Move our robot to the wall
            movement.linear.x = (
                (self.scan_front_dist - self.wall_distance) / (self.scan_max_range - self.wall_distance)
            ) * self.linear_speed

            self.move.publish(movement)

        # Stop the robot with an empty twist command
        movement = Twist()
        self.move.publish(movement)

        print("[ROBO-FORGER] Aligning with the wall...")
        # At this point we're close to the wall, and need to align our left side with it
        alignment_err = .1

        # While our bumpers differ by some error
        while alignment_err < abs(self.scan_front_bumper - self.scan_rear_bumper):
            movement.angular.z = pow(
                (self.scan_front_bumper - self.scan_rear_bumper) / self.scan_max_range, 2
            ) * self.angular_speed

            self.move.publish(movement)

        print("[ROBO-FORGER] Robot is aligned with the wall.")

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

        # Launch a command to move the arm
        self.vertical_move(y, continuous)

        # While the arm controller is handling that, control the robot's horizontal motion
        self.horizontal_move(x)

    # Move the robot's arm vertically
    # Continuous controls whether or not the arm stays flush with the wall
    def vertical_move(self, dist, continuous):
        pass

    # Move the robot horizontally
    def horizontal_move(self, dist):
        movement = Twist()
        movement.linear.x = self.linear_speed if dist > 0 else -self.linear_speed

        start_pos = self.odom_pose

        # While we haven't accomplished our goal distance
        while get_distance(start_pos, self.odom_pose) < abs(dist):
            # TODO: Calculate angular pid to make the robot stay aligned with the wall
            # Still need to enforce that we're a certain distance from the wall

            # Set the angular velocity to keep us aligned with the wall
            movement.angular.z = pow(
                (self.scan_front_bumper - self.scan_rear_bumper) / self.scan_max_range, 2
            ) * self.angular_speed

            self.move.publish(movement)
            self.rateLimit.sleep()

    def run(self):
        print("[ROBO-FORGER] Waiting for controller to be initialized")
        while not self.initialized():
            print("...")
            time.sleep(1)
        print("[ROBO-FORGER] Running.")
        rospy.spin()


if __name__ == "__main__":
    node = RoboForger()
    node.run()
