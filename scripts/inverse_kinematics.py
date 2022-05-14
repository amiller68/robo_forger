#!/usr/bin/env python3
import math
import moveit_commander # Import the moveit_commander, which allows us to control the arms
import rospy
import numpy as np

# A class that describes how the robot thinks about a point in 2D space
class Point(object):
    def __init__(self, coords, continuous=True):
        # A tuple of x, y coords
        self.coords = coords
        # Whether or not we should draw a line from this point to the next
        self.continuous = continuous
        pass


class RoboForger(object):
    
    def __init__(self):
        rospy.init_node("robo_forger")

        # Lengths of two joints used for IK
        self.l1 = 0.128
        self.l2 = 0.124

        # The interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")


    def compute_inverse_kinematics(self, x, y):

        q2 = math.acos(math.sin(math.acos((self.l1**2 + self.l2**2 - x**2 - y**2) / (2 * self.l1 * self.l2))))
        q1 = math.atan2(x, y) - math.atan2(self.l2 * math.cos(q2), self.l1)

        return q1, q2

    def print_valid_positions(self):
        for x in np.arange(0, self.l1 + self.l2, 0.01):
            for y in np.arange(0, self.l1 + self.l2, 0.01):
                try:
                    q1, q2 = self.compute_inverse_kinematics(x, y)
                    if q1 > math.radians(-103) and q1 < math.radians(90) and q2 > math.radians(-53) and q2 < math.radians(79):
                        print(x, y)

                except ValueError:
                    pass
                
    def run(self):
        
        q1, q2 = self.compute_inverse_kinematics(self.l2, self.l1)
        arm_joint_goal = [math.radians(90), q1, q2, -(q1 + q2)]
        print(arm_joint_goal)

        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()


if __name__ == "__main__":
    node = RoboForger()
    node.run()