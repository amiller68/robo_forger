#!/usr/bin/env python3
import csv
import cv2
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

path_prefix = os.path.dirname(__file__)
test_image = path_prefix + "/test_images/square.jpg"


'''
This class implements our computer vision component of our project

It reads in an image and converts it to an array of points, before publishing it to our RoboForger
'''


class ImageReader(object):
    def __init__(self):
        rospy.init_node("image_reader")

        # Declare a publisher for our generated drawings
        self.drawing_pub = rospy.Publisher("/robo_forger/drawing", Drawing, queue_size=10)

    # Read an image and convert it into a drawing (an array of points)
    def read_image(self, image):
        drawing = Drawing()
        
        # Converts an image to grayscale, identifies lines, creates array of line endpoints
        grayscale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        blurred_grayscale = cv2.GaussianBlur(grayscale,(5, 5),0)

        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blurred_grayscale, low_threshold, high_threshold)

        # HoughLinesP tranformation parameteres (Taken from Github so these parameters may need to be adjusted for our use)
        theta = np.pi / 180  # angular resolution in radians of the grid
        threshold = 15  # min # of interserctions
        min_line_length = 50  # min # of pixels making a line
        max_line_gap = 20  # max gap in pixels between line segments (this might need to change)


        lineArray = cv2.HoughLinesP(edges, 1, theta, threshold, np.array([]), min_line_length, max_line_gap)

        print(lineArray) # This is the array of endpoints
        return drawing

    def run(self):
        print("[IMAGE-READER] Reading in image: ", test_image)
        # For now this node just reads one image, publishes the resulting drawing, and returns
        with open(test_image, "r") as image:
            drawing = self.read_image(image)
            print("[IMAGE-READER] Publishing result")
            self.drawing_pub.publish(drawing)
        print("[IMAGE-READER] Run exiting.")


if __name__ == "__main__":
    node = ImageReader()
    node.run()

