#!/usr/bin/env python3

import math
import time

import moveit_commander
import numpy as np
import rospy
from robo_forger.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Setup parameters:
#   IRL: Should be set to True to add a movement delay and gripper weight compensation when running
#     on the physical Turtlebots
#   REGRIP: Should be set to True for the robot to open and close its gripper before running, in order
#     to regrip the writing utensil
IRL = True
REGRIP = False

# Set up extra delay and arm weight compensation for physical Turtlebot
EXTRA_DELAY = 5 if IRL else 0
WEIGHT_ANGLE = 10 if IRL else 0

# Offset parameters:
#   PUSH_OFFSET: Defines much the robot should push the marker into the wall
#   LIFT_OFFSET: Defines 
#   TOP_OFFSET: Defines 
PUSH_OFFSET =  0.002
LIFT_OFFSET = -0.030
TOP_OFFSET = 0.027

class RoboForgerIK(object):

    def __init__(self):

        # Initialize the inverse kinematics node
        rospy.init_node('robo_forger_ik')

        # Set a reasonable starting position
        self.curr_pos = (0.2, 0.2, 0.0)
        self.draw_pos = np.array([0.0, 1.0])
        self.board_dist = 0.3

        # Lengths of OpenMANIPULATOR arm joints
        self.l1 = 0.1302
        self.l2 = 0.124
        self.l3 = 0.214 # from gripper joint to marker tip

        # The interfaces to the group of joints making up the Turtlebot3 OpenMANIPULATOR arm and gripper
        self.move_group_arm = moveit_commander.MoveGroupCommander('arm')
        self.move_group_gripper = moveit_commander.MoveGroupCommander('gripper')

        # Set up publishers and subscribers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robo_forger/point', Point, self.recv_point)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # Initialize a default Twist message (all values 0)
        self.twist = Twist()

    # Executes the inverse kinematics algorithm, as computed for the OpenMANIPULATOR arm. The X parameter defines
    # distances to the left, Y defines distances up, and Z defines distances forward.
    def compute_inverse_kinematics(self, x, y, z):

        # The gripper takes up a certain horizontal length, so handle that
        totalDist = (z**2 + x**2)**0.5
        targetDistIK = totalDist-self.l3
        z *= targetDistIK/totalDist
        x *= targetDistIK/totalDist

        # Find the base angle
        q0 = math.atan2(x, z)

        # Convert z into in-plane horizontal distance for the two-joint IK
        z = (z**2 + x**2)**0.5

        # Squared distance from base joint to gripper wrist
        d_sqr = z**2 + y**2

        # Use law of cosines to find IK angles
        q1 = math.acos((self.l1**2 + d_sqr - self.l2**2) / (2 * self.l1 * math.sqrt(d_sqr)))
        q2 = math.acos((self.l1**2 + self.l2**2 - d_sqr) / (2 * self.l1 * self.l2))

        # Convert to robot arm coordinates. The -10.64 degree angle accounts for the rigid segment of
        #   the arm perpendicular to the length l1.
        q1 = math.radians(-10.64) + math.radians(90) - q1 - math.atan2(y, z)
        q2 = math.radians( 10.64) + math.radians(90) - q2

        return q0, q1, q2


    # Instructs the robot to drive for a specified amount of time, at a specified linear velocity. Can be
    #   used as a utility function for navigating the robot along the board.
    def drive(self, time, vel):

        # Set velocity, publish twist message, and give robot time to move
        self.twist.linear.x = vel
        self.twist_pub.publish(self.twist)
        rospy.sleep(time)

        # Stop the robot
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)


    # Helper function that moves the marker attached to the robot's end effector based on joint position parameters
    def move_marker_to_pose(self, a, b, c, d=None, delay=0):
        
        # Angle limits for Turtlebot3
        #   -162 < a < 162
        #   -102 < b <  90
        #    -54 < c <  79
        #   -102 < d < 117

        # Compute the gripper position perpendicular to the wall, accounting for the weight of the arm
        if d is None:
            d = -(b + c)
        d += math.radians(-WEIGHT_ANGLE)

        try:

            # Execute the motion
            s = time.time()
            self.move_group_arm.go([a, b, c, d], wait=True)
            self.move_group_arm.stop()
            e = time.time()

            time_taken = e - s

            # Pause for a short time
            sleep_time = delay + EXTRA_DELAY - time_taken
            sleep_time = max(sleep_time, 0) # Make sure time is positive
            rospy.sleep(sleep_time)
            
        except moveit_commander.exception.MoveItCommanderException:
            pass


    # Moves the marker attached to the robot's end effector based on an (x, y, z) position,
    #   with a specified number of waypoints
    def move_marker(self, x, y, z, num_waypoints=1):

        # Ensure we have at least one waypoint, and compute waypoint array based on current end effector position
        if num_waypoints < 1:
            num_waypoints = 1

        waypoints = np.linspace(self.curr_pos, [x, y, z], num_waypoints+1)[1:]

        # Go to each waypoint position
        for x1, y1, z1 in waypoints:

            # Attempt to use IK to compute joint positions
            try:
                q0, q1, q2 = self.compute_inverse_kinematics(x1, y1, z1)

            # If the position cannot be reached based on the robot's arm lengths
            except ValueError:
                print(f"Inverse kinematics computation shows that this (x, y, z) position cannot be reached by the OpenMANIPULATOR arm.")
                return

            # If IK computation gives angles outside of the arm's range, return
            q0_min = math.radians(-162)
            q0_max = math.radians(162)
            q1_min = math.radians(-103)
            q1_max = math.radians(90)
            q2_min = math.radians(-53)
            q2_max = math.radians(79)
            if q0 < q0_min or q0 > q0_max or q1 < q1_min or q1 > q1_max or q2 < q2_min or q2 > q2_max:
                print(f"Inverse kinematics computed angles {q0}, {q1}, and {q2}, which are outside the range of the OpenMANIPULATOR joints.")
                return

            # Move the marker based on the joint angles
            self.move_marker_to_pose(q0, q1, q2)

        # Update the current end effector position
        self.curr_pos = x, y, z


    # Swings the Turtlebot3 arm around to a good starting position, and then waits before closing the gripper
    def reset_arm_position(self):

        # Move the arm to a starting position
        self.move_marker(0.27, 0.2, 0.0, 1)

        # Define starting positions for the open/closed gripper positions
        gripper_joint_goal_open = [0.019, 0.019]
        gripper_joint_goal_closed = [-0.01, -0.01]

        if REGRIP:
            # Open the gripper, and give time for the robot to execute the action
            print('Opening grip')
            self.move_group_gripper.go(gripper_joint_goal_open, wait=True)
            self.move_group_gripper.stop()
            rospy.sleep(EXTRA_DELAY)

            # Close the gripper, and give time for the robot to execute the action
            print('Closing grip')
            self.move_group_gripper.go(gripper_joint_goal_closed, wait=True)
            self.move_group_gripper.stop()
            rospy.sleep(EXTRA_DELAY)

        print('Ready!')

    # Callback for the receipt of a point, which instructs the robot to move or draw to that point
    def recv_point(self, data):

        # Get point and compute distance to point
        pt = np.array([data.x, data.y])
        dist = np.linalg.norm(self.draw_pos - pt)

        print()
        print()
        print("dist: ", dist)
        print("pt: ", pt)

        # If the end effector is already aclose enough to the point, ignore the move
        if dist < 0.02:
            print('Close enough, ignoring move')
            self.draw_pos = pt
            return

        # Compute lift distance and draw distance
        lift_dist = self.board_dist + LIFT_OFFSET
        draw_dist = self.board_dist + PUSH_OFFSET + TOP_OFFSET*(1-pt[1])

        # Convert to robot-centric coordinates
        pt *= 0.2
        self.draw_pos *= 0.2

        # If this is a starting point, place the marker there; otherwise, draw to the point
        if data.start:
            print('Lifting')
            self.move_marker(lift_dist, self.draw_pos[1], self.draw_pos[0], 1)
            print('Going to start')
            self.move_marker(lift_dist, pt[1], pt[0], int(dist * 4))
            print('Placing')
            self.move_marker(draw_dist, pt[1], pt[0], 1)
        else:
            print('Drawing')
            self.move_marker(draw_dist, pt[1], pt[0], int(dist * 8))

        # Convert out of robot-centric coordinates
        self.draw_pos = pt*5


    # Reset the arm position, and then wait to receive points
    def run(self):
        self.reset_arm_position()
        rospy.spin()


if __name__ == "__main__":
    node = RoboForgerIK()
    node.run()
