#!/usr/bin/env python3
import math
import moveit_commander # Import the moveit_commander, which allows us to control the arms
import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist

# A class that describes how the robot thinks about a point in 2D space
class Point(object):

    def __init__(self, coords, continuous=True):

        # A tuple of x, y coords
        self.coords = coords

        # Whether or not we should draw a line from this point to the next
        self.continuous = continuous


class RoboForger(object):
    
    def __init__(self):

        # Initialize this node
        rospy.init_node("robo_forger")

        # Lengths of two joints used for IK
        self.l1 = 0.128
        self.l2 = 0.124
        self.la = 0.024

        self.l1_prime = math.sqrt(self.l1**2 + self.la**2)

        # Ranges of the angles of the two joints used for IK
        self.q1_min = math.radians(-103)
        self.q1_max = math.radians(90)
        self.q2_min = math.radians(-53)
        self.q2_max = math.radians(79)

        # The interfaces to the group of joints making up the Turtlebot3 OpenMANIPULATOR arm and gripper
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set up publisher to cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Initialize a default Twist message (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)
        

    # Executes the inverse kinematics algorithm, as computed for the OpenMANIPULATOR arm
    def compute_inverse_kinematics(self, x, y):

        # q2 = math.acos(math.sin(math.acos((self.l1**2 + self.l2**2 - x**2 - y**2) / (2 * self.l1 * self.l2))))
        # q1 = math.atan2(x, y) - math.atan2(self.l2 * math.cos(q2), self.l1)


        q2 = math.acos(math.sin(math.acos((self.l1_prime**2 + self.l2**2 - x**2 - y**2) / (2 * self.l1 * self.l2))))
        q1 = math.radians(90) - math.atan2(y, x) - math.asin(self.la / self.l1) - math.acos((x**2 + y**2 + self.l1_prime**2 - self.l2**2)/(2*self.l1_prime*math.sqrt(x**2 + y**2)))

        return q1, q2


    # Prints the arm positions that are computed as within the angle range of the OpenMANIPULATOR joints
    def print_valid_positions(self):

        # Iterate through the possible x and y positions of the arm, in 0.01 m increments
        for x in np.arange(0, self.l1 + self.l2, 0.01):
            for y in np.arange(0, self.l1 + self.l2, 0.01):
                try:
                    q1, q2 = self.compute_inverse_kinematics(x, y)
                    
                    # Only print the (x, y) position if it falls within the angle ranges of the arm joints
                    if q1 > self.q1_min and q1 < self.q1_max and q2 > self.q2_min and q2 < self.q2_max:
                        print(x, y)

                # A ValueError is raised if the (x, y) position cannot be reached at any joint angle
                except ValueError:
                    pass

    
    # Instructs the robot to drive for a specified amount of time, at a specified linear velocity
    def drive(self, time, vel):

        # Set velocity, publish twist message, and give robot time to move
        self.twist.linear.x = vel
        self.twist_pub.publish(self.twist)
        rospy.sleep(time)

        # Stop the robot
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)


    # Moves the marker to the specified (x, y) position
    def move_marker(self, x, y, num_waypoints):

        for i in range(num_waypoints):

            goal_y = self.y_curr + ((i + 1) * (y - self.y_curr) / num_waypoints)
            
            # Attempt to use IK to compute joint positions
            try:
                q1, q2 = self.compute_inverse_kinematics(x, goal_y)
            
            # If the position cannot be reached, return
            except ValueError:
                print(f"Inverse kinematics computation shows that this (x, y) position cannot be reached by the OpenMANIPULATOR arm.")
                return

            # If IK computation gives angles outside of the arm's range, return
            if q1 < self.q1_min or q1 > self.q1_max or q2 < self.q2_min or q2 > self.q2_max:
                print(f"Inverse kinematics computed angles {q1} and {q2}, which are outside the range of the OpenMANIPULATOR joints.")
                return
            
            # Set yaw to 90 degrees, and set the angles of the next two revolute joints based on IK computation.
            #   The end effector joint should be set to -(q1 + q2) to offset the other joint angles and keep it
            #   perpendicular to the wall.
            arm_joint_goal = [math.radians(90), q1, q2, -(q1 + q2)]

            # Execute the move to the joint goal, providing time for it to finish
            self.move_group_arm.go(arm_joint_goal, wait=True)
            self.move_group_arm.stop()
            rospy.sleep(3)

        self.y_curr = y


    # Swings the Turtlebot3 arm around to a good starting position, and then waits before closing the gripper
    def reset_arm_position(self):

        self.y_curr = 0.15
        
        # Define a good starting position for the arm and an open/closed position for the gripper
        q1, q2 = self.compute_inverse_kinematics(0.1, 0.15)
        arm_joint_goal = [math.radians(90), q1, q2, -(q1 + q2)]
        gripper_joint_goal_open = [0.019, 0.019]
        gripper_joint_goal_closed = [-0.01, -0.01]

        # Open the gripper, and give time for the robot to execute the action
        self.move_group_gripper.go(gripper_joint_goal_open, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(2)

        # Go to the specified arm position, and then wait for 5 seconds to give time to insert the marker
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

        # Close the gripper, and give time for the robot to execute the action
        self.move_group_gripper.go(gripper_joint_goal_closed, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(2)

    
    # Instructs the robot to draw a square; this function can be used for testing functionality
    def draw_square(self):

        self.move_marker(0.1, 0.05, 2)
        self.drive(5, 0.02)
        self.move_marker(0.1, 0.15, 2)
        self.drive(5, -0.02)

    
    def run(self):
        
        # self.reset_arm_position()

        self.y_curr = 0.15
        self.draw_square()


if __name__ == "__main__":
    node = RoboForger()
    node.run()