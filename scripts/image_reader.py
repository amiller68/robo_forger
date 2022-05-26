#!/usr/bin/env python3

import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robo_forger.msg import Point
import rospy
import numpy as np
import os

path_prefix = os.path.dirname(__file__)
test_image = path_prefix + '/test_images/leaf.png'


class ImageReader(object):
    """ This node implements our computer vision component of our project by reading an image and converting it to an array of points. """

    def __init__(self):

        # Initialize this node
        rospy.init_node("image_reader")
        
        # The following two lines can be uncommented when working with the live camera feed:
        # self.bridge = cv_bridge.CvBridge()
        # rospy.Subscriber('/camera/rgb/image_raw', Image, self.read_image)

        # Declare a publisher for our generated points
        self.point_pub = rospy.Publisher('/robo_forger/point', Point, queue_size=10)


    # Helper function that sorts a generated line array so that the lines are drawn in an order that
    #   requires minimal movement 
    def sortLineArray(self, lineArray):

        # Initialize sorted line array and endpoints of previous line
        lineArraySorted = np.copy(lineArray)
        prev_x, prev_y = -0.5, -0.5

        # For each line in the array, find the optimal next line to draw
        for i in range(len(lineArraySorted)):
            
            # Keep track of the closest line, its distance to the previous x and y positions, and its index
            closest_dist = np.inf
            closest_line = None
            closest_idx = -1

            # Search through the line array to find the nearest line to the previous x and y positions
            for j, line in enumerate(lineArray):

                x1, y1, x2, y2 = line

                # Check the distance to the starting endpoint of the line, as well as the distance
                #   to the starting endpoint if the line were to be flipped; update the closest distance
                #   if necessary
                dist = (prev_x - x1)**2 + (prev_y - y1)**2
                dist_flipped = (prev_x - x2)**2 + (prev_y - y2)**2

                if dist < closest_dist:
                    closest_dist = dist
                    closest_line = line
                    closest_idx = j

                if dist_flipped < closest_dist:
                    closest_dist = dist_flipped
                    closest_line = [x2, y2, x1, y1]
                    closest_idx = j

            # Update the previous x and y positions to the endpoint of the selected line, and add the selected line
            #   to the sorted line array
            prev_x = closest_line[2]
            prev_y = closest_line[3]
            lineArraySorted[i] = closest_line

            # Remove the selected line from the line array
            lineArray = np.delete(lineArray, closest_idx, axis=0)

        return lineArraySorted


    # Read an image and convert it into an array of lines
    def read_image(self, image):

        # The following line can be uncommented when working with the live camera feed:
        # image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # Converts an image to grayscale to simplify processing
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Blurs the image to remove noise that would cause false positive line detections
        blurred_grayscale = cv2.GaussianBlur(grayscale, (5, 5), 0)

        # Find the most prominent edges with the Canny algorithm
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blurred_grayscale, low_threshold, high_threshold)

        # The following three lines can be uncommented to show the edges found by cv2
        # cv2.imshow('image', image) # Original image
        # cv2.imshow('edges', edges) # Edges image
        # cv2.waitKey(0)

        # Set up HoughLinesP transformation parameters
        theta = np.pi / 180  # angular resolution in radians of the grid
        threshold = 30  # min # of interserctions
        min_line_length = 15  # min # of pixels making a line
        max_line_gap = 50  # max gap in pixels between line segments

        # HoughLines turns pixel-based edges into line segments defined by start and end points
        lineArray = cv2.HoughLinesP(edges, 1, theta, threshold, np.array([]), min_line_length, max_line_gap) 

        # Test points for drawing a square
        if False:
            lineArray = [
                [[           0,            0,            0, edges.shape[1]]],
                [[           0, edges.shape[1], edges.shape[0], edges.shape[1]]],
                [[edges.shape[0], edges.shape[1], edges.shape[0],            0]],
                [[edges.shape[0],            0,            0,            0]],
            ]
            lineArray = lineArray*10

        # Test points for a startup calibration routine
        if False:
            lineArray = [
                [[ 0, 0, 0, 0]],
                [[ 0, 0.5, 0, 0.5]],
                [[ 0, 1, 0, 1]],
                [[ 0.5, 1, 0.5, 1]],
                [[ 1, 1, 1, 1]],
                [[ 1, 0.5, 1, 0.5]],
                [[ 1, 0, 1, 0]],
                [[ 0.5, 0, 0.5, 0]],
            ]
            lineArray = lineArray*10
            lineArray = np.array(lineArray)
            lineArray = np.squeeze(lineArray, axis=1)
            lineArray = lineArray.astype(float)

        # Convert the lines from image coordinates to robot/drawing coordinates
        lineArray = np.array(lineArray)
        lineArray = np.squeeze(lineArray, axis=1)
        lineArray = lineArray.astype(float) / max(edges.shape)

        # Flip y axis
        lineArray[:, [1,3]] = 1 - lineArray[:, [1,3]]

        # Transform x values from [0, 1] to [-0.5, 0.5]
        lineArray[:, [0,2]] = lineArray[:, [0,2]] - 0.5

        # Sort line array to choose order of lines optimally
        lineArraySorted = self.sortLineArray(lineArray)

        # Publish the endpoints associated with each line
        print("[IMAGE-READER] Sending points over topic")
        for line in lineArraySorted:
            print("[IMAGE-READER] Sending line: ", line)
            x1, y1, x2, y2 = line
            self.point_pub.publish(Point(x=x1, y=y1, start=True))
            self.point_pub.publish(Point(x=x2, y=y2, start=False))
            rospy.sleep(1)


    # Retrieve the test image and call read_image
    def run(self):
        print("[IMAGE-READER] Reading in image...")
        img = cv2.imread(test_image)
        self.read_image(img)
        rospy.spin()


if __name__ == "__main__":
    node = ImageReader()
    node.run()