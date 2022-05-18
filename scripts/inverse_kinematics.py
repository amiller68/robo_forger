#!/usr/bin/env python3

import math
import moveit_commander # Import the moveit_commander, which allows us to control the arms
import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from robo_forger.msg import Point

# A class that describes how the robot thinks about a point in 2D space
class Point(object):

    def __init__(self, coords, continuous=True):

        # A tuple of x, y coords
        self.coords = coords

        # Whether or not we should draw a line from this point to the next
        self.continuous = continuous


class RoboForgerIK(object):

    def __init__(self):

        # Initialize this node
        rospy.init_node("robo_forger_ik")

        # Set a reasonable starting position
        self.curr_pos = (0.2, 0.2, 0.0)
        self.draw_pos = [0.0, 0.1]

        # Lengths of joints used for IK
        # Length of first joint
        self.l1 = 0.130
        # Length of second joint
        self.l2 = 0.124
        # Length from the gripper joint to the marker tip
        self.l3 = 0.230

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
        rospy.Subscriber('/robo_forger/point', Point, self.recv_point)

        # Initialize a default Twist message (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)


    # Executes the inverse kinematics algorithm, as computed for the OpenMANIPULATOR arm
    def compute_inverse_kinematics(self, x, y, z):
        # X left, Y up, Z forward

        # The gripper takes up a certain horizontal length, so handle that
        totalDist = (z**2 + x**2)**0.5
        targetDistIK = totalDist-self.l3
        z *= targetDistIK/totalDist
        x *= targetDistIK/totalDist

        # Find the base angle
        q0 = math.atan2(x, z)

        # Convert z into in-plane horizontal distance for the 2 joint IK
        z = (z**2 + x**2)**0.5

        # Distance from base joint to gripper wrist (squared)
        d_sqr = z**2 + y**2

        # Use law of cosines to find IK angles
        # print((self.l1**2 + d_sqr - self.l2**2) / (2 * self.l1 * math.sqrt(d_sqr)))
        q1 = math.acos((self.l1**2 + d_sqr - self.l2**2) / (2 * self.l1 * math.sqrt(d_sqr)))
        q2 = math.acos((self.l1**2 + self.l2**2 - d_sqr) / (2 * self.l1 * self.l2))

        # Convert to robot arm coordinates
        q1 = math.radians(-11) + math.radians(90) - q1 - math.atan2(y, z)
        q2 = math.radians( 11) + math.radians(90) - q2

        return q0, q1, q2


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


    def move_marker_to_pose(self, a, b, c, d=None, delay=4):
        # Angle limits
        # -162 < a < 162
        # -102 < b <  90
        #  -54 < c <  79
        # -102 < d < 117

        # The gripper angle can automatically be found
        # (It is always horizontal)
        if d is None:
            d = -(b + c)

        # The arm has some weight irl, so do a slight offset
        b += math.radians(-5)
        d += math.radians(-5)

        try:
            # Do the motion
            self.move_group_arm.go([a, b, c, d], wait=True)
            self.move_group_arm.stop()

            # Waiting seems necessary irl, so pause for a short time
            rospy.sleep(delay)
        except moveit_commander.exception.MoveItCommanderException:
            pass

    # Moves the marker to the specified (x, y, z) position
    def move_marker(self, x, y, z, num_waypoints=1):
        if num_waypoints < 1:
            num_waypoints = 1

        waypoints = np.linspace(self.curr_pos, [x, y, z], num_waypoints+1)[1:]

        for x1, y1, z1 in waypoints:
            # Attempt to use IK to compute joint positions
            try:
                q0, q1, q2 = self.compute_inverse_kinematics(x1, y1, z1)
            # If the position cannot be reached, return
            except ValueError:
                print(f"Inverse kinematics computation shows that this (x, y) position cannot be reached by the OpenMANIPULATOR arm.")
                return

            # If IK computation gives angles outside of the arm's range, return
            if q1 < self.q1_min or q1 > self.q1_max or q2 < self.q2_min or q2 > self.q2_max:
                print(f"Inverse kinematics computed angles {q1} and {q2}, which are outside the range of the OpenMANIPULATOR joints.")
                return

            self.move_marker_to_pose(q0, q1, q2)

        self.curr_pos = x, y, z


    # Swings the Turtlebot3 arm around to a good starting position, and then waits before closing the gripper
    def reset_arm_position(self):

        # self.move_marker(0.27, 0.1, 0.0, 1)
        self.curr_pos = (0.2, 0.2, 0.0)

        # Define a good starting position for the arm and an open/closed position for the gripper
        q0, q1, q2 = self.compute_inverse_kinematics(0.2, 0.2, 0.0)
        gripper_joint_goal_open = [0.019, 0.019]
        gripper_joint_goal_closed = [-0.01, -0.01]

        # Open the gripper, and give time for the robot to execute the action
        self.move_group_gripper.go(gripper_joint_goal_open, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(2)

        # Go to the specified arm position, and then wait for 5 seconds to give time to insert the marker
        self.move_marker_to_pose(q0, q1, q2)
        rospy.sleep(5)

        # Close the gripper, and give time for the robot to execute the action
        self.move_group_gripper.go(gripper_joint_goal_closed, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(2)


    # Instructs the robot to draw a square; this function can be used for testing functionality
    def draw_square(self):

        self.move_marker(0.1, 0.02, 1)
        self.drive(5, 0.02)
        self.move_marker(0.1, 0.2, 1)
        self.drive(5, -0.02)


    # Instructs the robot to draw a circle; this function can be used for testing functionality
    def draw_circle(self):
        for theta in np.linspace(0, np.pi*2, 32):
            z = math.cos(theta) * 0.05
            y = math.sin(theta) * 0.045
            self.move_marker(.2, y+.1, z, num_waypoints=1)

    def recv_point(self, data):
        # print('Got point %.3f %.3f %s' % (data.x, data.y, str(data.start)))

        dist = ((self.draw_pos[0]-data.x)**2 + (self.draw_pos[1]-data.y)**2)**0.5

        if data.start:
            print('Lift')
            self.move_marker(0.27, self.draw_pos[1], self.draw_pos[0], 1)
            print('Go to start')
            self.move_marker(0.27, data.y, data.x, 1)
            print('Place')
            self.move_marker(0.30, data.y, data.x, 1)
        else:
            print('Draw')
            self.move_marker(0.30, data.y, data.x, int(dist / 0.035))

        self.draw_pos = [data.x, data.y]

    def run(self):

        self.reset_arm_position()

        # self.draw_circle()
        # self.draw_square()
        rospy.spin()


if __name__ == "__main__":
    node = RoboForgerIK()
    node.run()
