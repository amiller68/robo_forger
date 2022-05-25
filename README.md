# Robo Forger

## WRITEUP:

### Project Description
*Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate.*

**Goal:** This project aims to have the turtlebot take an image fed to it (either from a file or live camera feed), use computer vision to identify line segments within the image, and have the turtlebot use its robot manipulator arm to draw this image on a wall. This goal requires the incorporation of multiple sensors, tools, and algorithms: live camera feed, LiDAR scanner, robot manipulator arm, computer vision (using OpenCV), and a custom built inverse kinematics algortihm.

**Successes:**
Computer Vision: The turtlebot is successfully able to take an image, identify line segments within that image, and then convert those segments into points that can be sent to the robot.

Inverse Kinematics: The turtlebot is able to receive a set of points (taken from the computer vision algorithm) and use its manipulator arm to draw those line segments on a vertical surface next to it. The robot is also able to use its LiDAR scanner to understand its distance from the wall and adjust the angles of its motors to align properly with the wall.

The computer vision algorithm is fed the orange maple leaf on the left. The turtlebot then outputs the drawing as shown below.
<img src="https://github.com/amiller68/robo_forger/blob/31b45fd845ce5a1ae888667ac7faeb9bea0d9af1/scripts/test_images/leaf.png" alt="Maple Leaf Image" width="300"/><img src="https://github.com/amiller68/robo_forger/blob/520d4eb5c19f63b976a737f161a9c06c8939f991/ezgif-4-180f11597a.gif" alt="Maple Leaf Gif" width="520"/>

**Main Components:**
- Drawing Implement Attachment: This implement attachment consists of a 3D-printed holder, spring loaded pen holder, and a thin-tipped pen. The 3D-printed holder is designed to allow for the robot manipulator arm grip to properly grab the pen. The spring loaded pen holder is inserted into the 3D-printed holder and allows for the pen to have a bit "give" when drawing on the wall. This increases the liklihood of the pen making contact with the drawing surface for the entirety of the drawing.
- Computer Vision: The computer vision algorithm takes in an image and converts it into an array of points that is then published to the turtlebot.
- LiDAR: The LiDAR sensor is used to measure the distance between the robot and the wall - this is then incorporated into the kinematics algorithm in order to determine the distance to which the arm should extend the pen.
- Inverse Kinematics Algorithm: The inverse kinematics algorithm receives a point in 3D space relative to the base of the turtlebot's arm, and computes the arm joint angles necessary to move the pen tip to that 3D point.
- Communication: The computer vision algorithm determines the line endpoint positions, and pipes these through the inverse kinematics algorithm to determine where to put the robot arm. The LiDAR sensor provides an offset to these point positions right before the arm configuration is determined.

**Individual Contributions:**
- Samir Rajani: Worked on the inverse kinematics algorithm for computing joint angles in the forward/reverse and up/down plane, implemented the drawing order for the line array to decide the order and direction in which lines should be drawn to reduce drawing time and improve accuracy, implemented initial version of alignment.
- Rory Butler: Worked on the extension of the inverse kinematics to 3D space by computing left/right angles and handling the projection into the 2D plane; implemented correction terms for inverse kinematics on the physical Turtlebot; helped develop the computer vision component and initial code to send detected points to the inverse kinematics algorithm.
- Alex Miller: Worked on code for aligning the Turtlebot, coming up with ways to deal with the inaccuracy of the LiDAR, and integrated the alignment, computer vision, and inverse kinematics components together.
- Nick Auen: Worked primarily on computer vision component, including edge detection and conversion to a line array, getting versions working that both used the live feed from the robot camera and with images being fed directly; filled out the initial layout and major sections of the readme and presentations.

### System Architecture
*Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components.*

- Computer Vision: The computer vision algorithm receives an image from either the camera or a local image file. The image is first converted into grayscale and then put through a Gaussian blur filter. Edges in the image are identified using the Canny function in OpenCV which takes an image, denoises it, finds its intensity gradient, and then removes all pixels that are not on edges. Threshold values within the Canny function define the lower and upper bounds for intensity gradients that are considered edges. An array of lines is then created using the Hough Lines tranformation function within OpenCV (*HoughLinesP()*). The Hough Lines transformation identifies straight lines within an image. The parameters for the Hough Lines transformation depend on the image being used and have been hand-tuned for the maple leaf image. This array of lines is then sorted in order to group lines that are closer to one another.
<img src="https://github.com/amiller68/robo_forger/blob/31b45fd845ce5a1ae888667ac7faeb9bea0d9af1/scripts/test_images/leaf.png" alt="Maple Leaf Image" width="300"/><img width="600" alt="Screen Shot 2022-05-25 at 3 33 54 PM" src="https://user-images.githubusercontent.com/102747072/170362215-ed82eb43-41f6-467f-81ce-b7586a0356e0.png">
- Inverse Kinematics Algorithm:


### Execution
*Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.*
1. Turn on the turtlebot3
2. Terminal 1
    1. Run "roscore" on your machine
3. Terminal 2
    1. SSH into the turtlebot3
    2. Run "set_ip (insert last 3 digits of your ip)"
    3. Run "bringup"
4. Terminal 3
    1. SSH into the turtlebot3 in another terminal
    2. Run "set_ip (insert last 3 digits of your ip)"
    3. Run "bringup_cam"
5. Terminal 4
    1. Run "roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch"
6. Terminal 5
    1. Run "roslaunch turtlebot3_manipulation_moveit_config move_group.launch"
7. Terminal 6
    1. Run "rosrun robo_forger main.py"
8. Wait for the robot to be aligned. 
   1. If `ALIGN_ROBOT` in `main.py` is set to `True`, then the robot will attempt to align itself with the wall in front of it.
   2. Otherwise, the robot controller will state that it is aligned and prepare to start drawing points.
   3. Wait for the process running `main.py` to print `[ROBO FORGER] Ready to Forge` before moving on the next step
9. Terminal 7
    1. Run "rosrun robo_forger image_reader.py"
10. At this point, the robot should start drawing our test image on the wall to its left.


### Challenges, Future Work, & Takeaways
*These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways).*

**Challenges:**
- Keeping the robot manipulator arm aligned properly with the wall over the course of drawing an entire image proved to be particuarly complex. Keeping the tip of the writing implement the proper distance from the wall (not too close that it cannot move smoothly, and not too far that it does not draw a line) as the angle of the manipulator arm changed was made difficult given: the change in angle of the tip of the writing implement and the effect that the weight of the arm itself had on its own movement.
-

**Future Work:**
- Setting the robot up with a server that continually serves the robot pictures to draw. This server could accept and queue images submitted by observers.
- Installing this system as an interactive exhibit in MAAD
- Working on computer vision to remove the presence of double lines.
- Testing the robot with different drawing tools and canvases

**Takeaways:**
- The weight of the manipulator arm itself can affect the accuracy and movement of the joints within the arm.
- Line segmentation parameters may need to be tuned depedning on the complexity of the image being analyzed.
- Lag on the *intro-robo* network can greatly influence not just the speed of the robot but the accuracy of its movements.


## Running:
- Make sure you're running `roscore`
- Launch the robot's manipulation stuff
  - `roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch`
- Launch the controller for the arm
  - `roslaunch turtlebot3_manipulation_moveit_config move_group.launch`
