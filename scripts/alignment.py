#!/usr/bin/env python3
# TOPICS:
#   cmd_vel: publishing, used for setting robot velocity
#   scan   : subscribing, used to detect where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Alignment(object):
    """ This node instructs the robot to align itself with a wall. """
    # Constructor parameters
    #   k_p_ang:         proportional control factor controlling how much the error in
    #                      angle affects the angular velocity 
    def __init__(self, k_p_ang=0.0004):        
        # Start rospy node.
        rospy.init_node("wall_alignment")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # Initialize constructor parameters.
        self.k_p_ang = k_p_ang

    # Determine nearest object by looking at scan data from all angles
    #   the robot, set velocity based on that information, and
    #   publish to cmd_vel.
    def process_scan(self, data):
        # Initialize nearest_index and nearest_distance, which will keep
        #   track of the angle and distance to the object nearest to the
        #   robot.
        nearest_index = -1
        nearest_distance = 10 # Maximum LiDAR distance is 4.1m, so init value is safe.

        # Iterate through data.ranges, which contains the nearest object
        #   at each degree increment. If the value is non-zero, there is
        #   an object in that direction.
        for i in range(70, 110):
            if data.ranges[i] > 0.0 and data.ranges[i] < nearest_distance:
                nearest_index = i
                nearest_distance = data.ranges[i]

        # If no object was found within the LiDAR range, stop the robot.
        if (nearest_distance == 10):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            return

        # Calculate the discrepancy between the robot's distance and angle
        #   and the desired wall distance and the angle (90 degrees),
        #   respectively. The ternary operator is used to convert the range
        #   of angles so that proportional control makes the robot turn in the
        #   right direction.
        error_angle = nearest_index - 90

        if error_angle == 0:
            print("Aligned!")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            rospy.signal_shutdown("Aligned!")

        # Set velocity based on the proportional control mechanism.
        self.twist.angular.z = self.k_p_ang * error_angle

        # Clamp angular speed to 1.82 rad/s to ensure we don't exceed the maximum velocity of the Turtlebot.
        self.twist.angular.z = min(1.82, max(self.twist.angular.z, -1.82))
    
        # Publish Twist message to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node with default constructor parameters and run it.
    node = Alignment()
    node.run()