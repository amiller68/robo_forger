#!/usr/bin/env python3
import csv
import math
import sys
import threading
import time

import rospy
import numpy as np
import os

# A class that describes how the robot thinks about a point in 2D space
class Point(object):
    def __init__(self, coords, continuous=True):
        # A tuple of x, y coords
        self.coords = coords
        # Wether or not we should draw a line from this point to the next
        self.continuous = continuous
        pass


class RoboForger(object):
    def __init__(self):
        rospy.init_node("robo_forger")

        self.points = None

        self.recognize_image()
        pass

    # Recognize image and load it into a poin
    def recognize_image(self):
        # For now it just loads a square
        self.points = list(map(
            lambda x: Point(x),
            [(0, 0), (0, 5), (5, 5), (5, 0)]
        ))

    def draw_image(self):
        pass


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RoboForger()
    node.run()