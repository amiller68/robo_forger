# Robo Forger

## WRITEUP:

### Project Description
Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate.

**Goal:** This project aims to have the turtlebot take an image fed to it (either from a file or live camera feed), use computer vision to identify line segments within the image, and have the turtlebot use its robot manipulator arm to draw this image on a wall. This goal requires the incorporation of multiple sensors, tools, and algorithms: live camera feed, LiDAR scanner, robot manipulator arm, computer vision (using OpenCV), and a custom built inverse kinematics algortihm. 

**Successes:**

**Main Components:**
- Drawing Implement Attachment:
- Computer Vision:
- LiDAR:
- Inverse Kinematics Algorithm:


### System Architecture
Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components.

### Execution
Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.

### Challenges, Future Work, & Takeaways
These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways).

**Challenges:**
- Keeping the robot manipulator arm aligned properly with the wall over the course of drawing an entire image proved to be particuarly complex. Keeping the tip of the writing implement the proper distance from the wall (not too close that it cannot move smoothly, and not too far that it does not draw a line) as the angle of the manipulator arm changed was made difficult given: the change in angle of the tip of the writing implement and the effect that the weight of the arm itself had on its own movement. 
- 

**Future Work:**
- (Insert Future Work)

**Takeaways:**
- The weight of the manipulator arm itself can affect the accuracy and movement of the joints within the arm.

## Running:

- Make sure you're running `roscore`
- Launch the robot's manipulation stuff
  - `roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch`
- Launch the controller for the arm
  - `roslaunch turtlebot3_manipulation_moveit_config move_group.launch`
