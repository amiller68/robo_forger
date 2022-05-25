#!/usr/bin/env python3

import math
import time

import moveit_commander
import numpy as np
import rospy
from robo_forger.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

IRL = True
REGRIP = False

EXTRA_DELAY = 3 if IRL else 0
WEIGHT_ANGLE = 10 if IRL else 0

PUSH_OFFSET =  0.002
LIFT_OFFSET = -0.030
TOP_OFFSET = 0.027

class RoboForgerIK(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node('robo_forger_ik')

        # Set a reasonable starting position
        self.curr_pos = (0.2, 0.2, 0.0)
        self.draw_pos = np.array([0.0, 1.0])
        self.board_dist = 0.3

        # Lengths of joints used for IK
        # Length of first joint
        self.l1 = 0.1302
        # Length of second joint
        self.l2 = 0.124
        # Length from the gripper joint to the marker tip
        # self.l3 = 0.1466 # gripper front
        self.l3 = 0.214 # pen

        # The interfaces to the group of joints making up the Turtlebot3 OpenMANIPULATOR arm and gripper
        self.move_group_arm = moveit_commander.MoveGroupCommander('arm')
        self.move_group_gripper = moveit_commander.MoveGroupCommander('gripper')

        # Set up publishers and subscribers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robo_forger/point', Point, self.recv_point)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # Initialize a default Twist message (all values 0)
        self.twist = Twist()

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
        q1 = math.radians(-10.64) + math.radians(90) - q1 - math.atan2(y, z)
        q2 = math.radians( 10.64) + math.radians(90) - q2

        return q0, q1, q2

    # Instructs the robot to drive for a specified amount of time, at a specified linear velocity
    def drive(self, time, vel):
        # Set velocity, publish twist message, and give robot time to move
        self.twist.linear.x = vel
        self.twist_pub.publish(self.twist)
        rospy.sleep(time)

        # Stop the robot
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)

    def move_marker_to_pose(self, a, b, c, d=None, delay=0):
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
        # b += math.radians(-WEIGHT_ANGLE)
        # c += math.radians(-WEIGHT_ANGLE)
        d += math.radians(-WEIGHT_ANGLE)

        try:
            # Do the motion
            s = time.time()
            self.move_group_arm.go([a, b, c, d], wait=True)
            self.move_group_arm.stop()
            e = time.time()

            time_taken = e-s

            # Waiting seems necessary irl, so pause for a short time
            sleep_time = delay + EXTRA_DELAY - time_taken
            sleep_time = max(sleep_time, 0) # Make sure not negative!
            rospy.sleep(sleep_time)
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
            q1_min = math.radians(-103)
            q1_max = math.radians(90)
            q2_min = math.radians(-53)
            q2_max = math.radians(79)
            if q1 < q1_min or q1 > q1_max or q2 < q2_min or q2 > q2_max:
                print(f"Inverse kinematics computed angles {q1} and {q2}, which are outside the range of the OpenMANIPULATOR joints.")
                return

            self.move_marker_to_pose(q0, q1, q2)

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

    def recv_point(self, data):
        # print('Got point %.3f %.3f %s' % (data.x, data.y, str(data.start)))

        pt = np.array([data.x, data.y])
        dist = np.linalg.norm(self.draw_pos - pt)

        print()
        print()
        print("dist: ", dist)
        print("pt: ", pt)

        # if False:#dist <= 0.001:
        #     print('Close enough, ignoring move')
        #     self.draw_pos = pt
        #     return

        if dist < 0.02:
            print('Close enough, ignoring move')
            self.draw_pos = pt
            return

        lift_dist = self.board_dist + LIFT_OFFSET
        draw_dist = self.board_dist + PUSH_OFFSET + TOP_OFFSET*(1-pt[1])

        # Convert to robot-centric coordinates
        pt *= 0.2
        self.draw_pos *= 0.2

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

    def process_scan(self, data):
        return
        # Get the lidar distances
        d = data.ranges
        # Filter out bogus 0's
        d = list(filter(lambda x:x!=0, d))
        # Filter out bogus infinities
        d = list(filter(lambda x:x!=np.inf, d))

        if len(d) == 0:
            return

        # Find the closest distance
        d = min(d)
        print('d', d)

        # Weighted average with previous distance
        weight = 0.1
        self.board_dist = (self.board_dist * (1-weight)) + (d * weight)

    def run(self):
        self.reset_arm_position()
        rospy.spin()

if __name__ == "__main__":
    node = RoboForgerIK()
    node.run()
