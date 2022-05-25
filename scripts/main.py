#!/usr/bin/env python3

import rospy
from math import inf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class Alignment(object):
    """ This node instructs the robot to align itself with a wall. """
    # Constructor parameters
    #   k_p_ang:         proportional control factor controlling how much the error in
    #                      angle affects the angular velocity 
    def __init__(self, k_p_ang=0.01):        
        # Start rospy node
        rospy.init_node("wall_alignment")

        # Set class initialization to False
        self.class_initialized = False

        # Class state variables
        self.linear_speed = .2  # m/s
        self.angular_speed = -.4  # rad/s
        self.goal_wall_distance = .35  # m
        self.aligned = False  # Whether we're ready to start drawing

        # General state variables to hold
        self.odom_pose = None  # our current odometry position (x, y) coords
        self.scan_ranges = None  # our current scan state

        # Variables that access specific scan range variables
        self.scan_max_range = None  # Our maximum scan range
        self.scan_front_dist = None  # How far the object directly in front of us is
        self.scan_left_dist = None  # How far whatever is to our left is
        self.scan_front_bumper = None  # A scan range for our front bumper (for keeping the robot aligned)
        self.scan_rear_bumper = None  # A scan range for our rear bumper (for keeping the robot aligned)

        rospy.Subscriber('/odom', Odometry, self.set_odom_pose)

        # Rospy Params
        self.queue_size = 20  # The size of our message queues
        self.rate_limit = rospy.Rate(10)  # How often we publish messages (10 Hz), utilize with self.rate_limit.sleep()

        # Scan subscriber
        rospy.Subscriber('/scan', LaserScan, self.set_scan_data)

        # Velocity Publisher
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=self.queue_size)

        self.class_initialized = True
        

    def initialized(self):
        return self.class_initialized \
            and self.scan_ranges is not None \
            and self.odom_pose is not None \


    def set_scan_data(self, msg):
        # Filter our ranges for any bad readings and set them to the max range
        ranges = list(map(lambda x: x if (x > 0 and x != inf) else msg.range_max, msg.ranges))
        self.scan_max_range = msg.range_max
        self.scan_ranges = ranges


        # Get the distance right in front of us
        self.scan_front_dist = ranges[0]

        # Get the distance to our left
        self.scan_left_dist = ranges[90]

        # Our bumpers for wall following are on the left side of the robot

        # Get the distance of our front bumper
        self.scan_front_bumper = ranges[45]

        # Get the distance of rear bumper
        self.scan_rear_bumper = ranges[135]

    def set_odom_pose(self, msg):
        # Extract the pose from the odometry message
        position = msg.pose.pose.position
        self.odom_pose = (position.x, position.y)

    # Align our robot with the wall and its arm with our origin
    # Assumes the robot is at least kinda facing a wall
    def align(self):
        movement = Twist()

        print("[ROBO-FORGER] Approaching a wall...")
        # While We're not close to a wall to draw on
        while self.goal_wall_distance < self.scan_front_dist:
            # Move our robot to the wall
            movement.linear.x = max(
                (self.scan_front_dist - self.goal_wall_distance) / (self.scan_max_range - self.goal_wall_distance) * self.linear_speed,
                0.001
            )
            self.move.publish(movement)

        # Stop the robot with an empty twist command
        movement = Twist()
        self.move.publish(movement)

        print("[ROBO-FORGER] Aligning with the wall...")

        # At this point we're close to the wall, and need to align our left side with it

        # TODO: Tweak this to be accurate
        alignment_err = .05

        # While our bumpers differ by some error
        while alignment_err < abs(self.scan_front_bumper - self.scan_rear_bumper):
            movement.angular.z = abs(self.scan_front_bumper - self.scan_rear_bumper) / self.scan_max_range * self.angular_speed
            self.move.publish(movement)

        # Stop the robot with an empty twist command
        movement = Twist()
        self.move.publish(movement)

        print("[ROBO-FORGER] Robot is aligned with the wall.")

    def run(self):
   
        try:
            # Wait for the controller to be initialized
            print("[ROBO-FORGER] Waiting for controller to be initialized")
            while not self.initialized():
                print("...")
                rospy.sleep(1)

            # Align the robot
            print("[ROBO-FORGER] Aligning robot with nearest wall in sight...")
            self.align()
            return

        except KeyboardInterrupt:
            print("[ROBO-FORGER] Exit signalled...")
            return

if __name__ == '__main__':
    # Declare a node with default constructor parameters and run it.
    node = Alignment()
    node.run()