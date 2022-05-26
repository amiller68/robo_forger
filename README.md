# Robo Forger

## WRITEUP:

### Project Description
*Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate.*

**Goal:** This project aims to have the turtlebot take an image fed to it (either from a file or live camera feed), use computer vision to identify line segments within the image, and use the OpenMANIPULATOR arm to draw this image on a wall. This goal requires the incorporation of multiple sensors, tools, and algorithms: the LiDAR scanner (for alignment), the OpenMANIPULATOR arm, computer vision (using OpenCV), and a custom inverse kinematics algortihm.

**Why This is Interesting:** Inverse kinematics is a far more difficult problem than forward kinematics, and drawing on a flat surface while being able to lift up and put down a marker requires use of all four degrees of freedom afforded by the OpenMANIPULATOR arm. The relatively limited range of the OpenMANIPULATOR arm and the imprecision of its movements, along with network lag and the weight of a physical drawing implement, also make drawing an image using a physical turtlebot a daunting task.

**Successes:**

- Computer Vision: The turtlebot is successfully able to take an image, identify line segments within that image, and then convert those segments into points that can be sent to the robot, performing a coordinate transformation to place these points within the range of the robot's arm. The line segments are also ordered such that minimal movement is required between finishing one line and starting the next.

- Inverse Kinematics: The turtlebot is able to receive a set of points (taken from the computer vision algorithm) and use its manipulator arm to draw those line segments on a vertical surface next to it using a custom 3 DOF inverse kinematics solver, lifting its arm between non-adjacent line segment endpoints. It is capable of interpolating between line endpoints using waypoints, which ensures lines remain straight. The robot is also able to use its LiDAR scanner to understand its distance from the wall and adjust the angles of its motors to align properly with the wall, but this is contingent on network lag and the precision of the LiDAR; manual adjustments are in general necessary.

**Demo:** The computer vision algorithm is fed the red maple leaf on the left. The turtlebot then outputs the drawing as shown below.
<img src="https://github.com/amiller68/robo_forger/blob/31b45fd845ce5a1ae888667ac7faeb9bea0d9af1/scripts/test_images/leaf.png" alt="Maple Leaf Image" width="300"/><img src="https://github.com/amiller68/robo_forger/blob/520d4eb5c19f63b976a737f161a9c06c8939f991/ezgif-4-180f11597a.gif" alt="Maple Leaf Gif" width="520"/>

**Relationship Between Components:**
- Drawing Implement Attachment: Throughout the course of this project, we devised multiple drawing implements, including a whiteboard marker holder made using tape and one of the tubes from the Q-learning project, a 3D-printed holder attached to a spring-loaded pen holder for a thin-tipped pen, and a 3D-printed sharpie holder. The 3D-printed holders for the latter two drawing implements are designed to allow for the robot manipulator arm grip to properly grab the pen. The spring-loaded pen holder is inserted into the 3D-printed holder and allows for the pen to have a bit "give" when drawing on the wall. This increases the likelihood of the pen making contact with the drawing surface for the entirety of the drawing. In our testing, we primarily used the whiteboard marker, but we have also found success using the other tools.
- LiDAR Alignment: The LiDAR sensor is used to measure the align the robot so that it is parallel to the wall, but manual adjustments are typically necessary due to LiDAR imprecision.
- Computer Vision: The computer vision algorithm takes in an image and converts it into an array of points that is then published to the turtlebot.
- Inverse Kinematics Algorithm: The inverse kinematics algorithm receives a point in 3D space relative to the base of the turtlebot's arm from the computer vision component and computes the arm joint angles necessary to move the pen tip to that 3D point using a custom IK solver.

**Individual Contributions:**
- Samir Rajani: Worked on the inverse kinematics algorithm for computing joint angles in the forward/reverse and up/down plane, implemented the drawing order for the line array to decide the order and direction in which lines should be drawn to reduce drawing time and improve accuracy, implemented initial versions of alignment and waypoints.
- Rory Butler: Worked on the extension of the inverse kinematics to 3D space by computing left/right angles and handling the projection into the 2D plane; implemented correction terms for inverse kinematics on the physical Turtlebot; helped develop the computer vision component and initial code to send detected points to the inverse kinematics algorithm.
- Alex Miller: Worked on code for aligning the Turtlebot, coming up with ways to deal with the inaccuracy of the LiDAR, and integrated the alignment, computer vision, and inverse kinematics components together.
- Nick Auen: Worked primarily on computer vision component, including edge detection and conversion to a line array, getting versions working that both used the live feed from the robot camera and with images being fed directly; filled out the initial layout and major sections of the readme and presentations.

### System Architecture
*Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components.*

- Alignment: The robot can autonomously align itself with the wall using LiDAR, though manual corrections may be necessary. This component is implemented in `alignment.py`. The callback for the scan topic searches through the 70 to 110 degree range of LiDAR data, which corresponds roughly with the left side of the robot, and finds the degree value with the smallest distance. The robot then attempts to position this distance at 90 degrees, using proportional control to set its angular velocity. The program terminates when the robot is aligned with the wall.

- Computer Vision: Our computer vision component is implemented in `image_reader.py`. The computer vision algorithm receives an image from either the camera or a local image file in the `read_image()` function; the code is currently configured for local images. The image is first converted into grayscale and then put through a Gaussian blur filter. Edges in the image are identified using the Canny function in OpenCV, which takes an image, denoises it, finds its intensity gradient, and then removes all pixels that are not on edges. Threshold values within the Canny function define the lower and upper bounds for intensity gradients that are considered edges. An array of lines is then created using the Hough Lines tranformation function within OpenCV (`HoughLinesP()`). The Hough Lines transformation identifies straight lines within an image. The parameters for the Hough Lines transformation depend on the image being used and have been hand-tuned for the maple leaf image. This array of lines is then sorted in order to group lines that are closer to one another, which is implemented in the `sortLineArray()` function. To do this, the robot repeatedly selects a line and then searches for the line endpoint nearest to the end of that line. For a simple test drawing consisting of contiguous lines, this approximately doubles the speed of drawing. Finally, coordinate transformations are performed on the data to prevent the image from being flipped when drawing and to ensure that an x value of 0 corresponds with the center of the axis. To send lines to the inverse kinematics code, a custom `Point` message is published for each line endpoint, which consists of floating point x and y coordinates, as well as a Boolean start to indicate whether this is the starting point or ending point of a line segment.
<img src="https://github.com/amiller68/robo_forger/blob/31b45fd845ce5a1ae888667ac7faeb9bea0d9af1/scripts/test_images/leaf.png" alt="Maple Leaf Image" width="300"/><img width="600" alt="Screen Shot 2022-05-25 at 3 33 54 PM" src="https://user-images.githubusercontent.com/102747072/170362215-ed82eb43-41f6-467f-81ce-b7586a0356e0.png">

- Inverse Kinematics: The custom inverse kinematics code is implemented in `inverse_kinematics.py`. Parameters are set up to specify the gripper angle that compensates for the weight of the writing utensil, whether the robot is already holding the writing utensil, and offsets that define how much the arm pushes into the wall when writing, how much it moves off the wall when moving between non-contiguous lines, and a correction term that tells the arm to back off the wall slightly at the top of its drawing area. The constructor for the `RoboForgerIK` class sets up lengths of the OpenMANIPULATOR arm joints, the starting position of the robot, and the appropriate publishers and subscribers. The function `compute_inverse_kinematics` implements a custom 3 DOF IK algorithm which computes a horizontal distance before projecting into the plane of the whiteboard to compute 2D IK in the forward/back and up/down plane. The `move_marker` function moves the marker to a specified (x, y, z) position with a particular number of waypoints by interpolating between the line's endpoints and invoking `compute_inverse_kinematics`, ensuring the computed joint angles are within the ranges of the OpenMANIPULATOR arm before invoking `move_marker_to_pose` to execute the move. It also uses and updates the `curr_pos` value, which is necessary for computing the linear interpolation. The `move_marker_to_pose` function accepts joint angles for the arm, and it computes the gripper joint angle to be perpendicular to wall based on the angles of the other joints, accounting for the weight of the writing implement. It then executes the motion on the arm, with some delay to allow time for the action to finish.


### Execution
*Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.*
1. Turn on the turtlebot3
2. Terminal 1
    1. Run "roscore" on your machine
3. Terminal 2
    1. SSH into the turtlebot3
    2. Run "set_ip (insert last 3 digits of your ip)"
    3. Run "bringup"
5. Terminal 4
    1. Run "roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch"
6. Terminal 5
    1. Run "roslaunch turtlebot3_manipulation_moveit_config move_group.launch"
7. Terminal 6 (OPTIONAL: alignment is generally imprecise due to LiDAR limitations)
    1. Place the robot so that it is roughly facing the left wall (within about 30 degrees or so)
    2. Run "rosrun robo_forger alignment.py"
    3. Wait for the program to terminate, at which point the robot should be aligned with the wall
8. Terminal 7
    1. Run "rosrun robo_forger inverse_kinematics.py", ensuring the REGRIP parameter is set to True in the code if the robot is not already holding the writing utensil.
    2. If REGRIP is set to True, you will have a few seconds after the robot's gripper opens to insert the writing utensil before it closes again.
9. Terminal 8 
    1. Run "rosrun robo_forger image_reader.py"
    2. At this point, the robot should start drawing our test image on the wall to its left.
    3. The distance to the drawing surface must be quite precise (on the order of millimeters), so make adjustments to the position of the drawing surface or the robot until you are satisfied with the drawing.


### Challenges, Future Work, & Takeaways
*These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways).*

**Challenges:**
Keeping the robot manipulator arm aligned properly with the wall over the course of drawing an entire image proved to be particuarly complex. Pushing too far into the wall increased the normal force on the writing implement, and the increased friction would prevent the arm from drawing smoothly. However, being too far out of the wall risked the robot's being unable to draw anything at all. We tried out a variety of hardware solutions, such as using a 3D-printed spring-loaded writing implement that had more "give" to it than a traditional pen, using a 3D-printed sharpie holder that would prevent the pen from moving around when it was pushed into the wall, and adjusting the drawing angle of the writing utensil. We eventually found the best solution to be the combination of a whiteboard marker and a correction term that made the robot push slightly less into the wall at the top of its drawing area than at the bottom. Another challenge we ran into was that the robot's lines were often disjointed, and the slow speed of drawing made testing a tedious process. To correct for this, we wrote a function that determined the best next line to draw from the line array generated from our computer vision code, flipping the line if necessary. We also implemented functionality so that the robot would not pick up and put down its pen again if the point it received was sufficiently close to where the arm currently was, allowing lines that shared endpoints to be drawn in sequence, fluidly and quickly. Finally, we solved the issue that longer lines weren't guaranteed to be straight by implementing waypoints, which would linearly interpolate between endpoints depending on their distance. 

**Future Work:**
Future work might include setting up a server that continually serves the robot pictures to draw. This server could accept and queue images submitted by observers, adding human interaction to the robot's functionality. Our robot's system could also be installed as an interactive exhibit (such as in the MADD Center), with full support for using the robot's camera data to accept hand-drawn images (rather than directly feeding in images). We could also work on improve the computer vision component of the project to remove the presence of double lines, which are formed for some images during line segment generation. Finally, we might test the robot with different drawing tools and canvases, possibly devising a formula to determine the correction factor for the arm's gripper angle depending on the weight of the writing utensil. We might also experiment with deriving the inverse kinematics numerically, rather than analytically, by computing the forward kinematics and then taking the inverse of an approximation (e.g. spline approximation) of these equations. Finally, we could extend the range of the robot's canvas by incorporating horizontal movement, using the robot's odometry to gauge distances.

**Takeaways:**
- The weight of the OpenMANIPULATOR arm itself, as well as any objects it is holding in the gripper, can affect the accuracy and movement of the joints within the arm. In general, the arm performs quite differently from in simulation, and there is no substitute for repeated tests on a physical robot. Also, even if the arm can reach a point in 3D space based on the angle ranges of its joints, these joints may, for example, require the arm to hit the robot's chassis, and thus movement to the position might not be possible in the real world.
- Parameters in computer vision, such as in line segment detection, generally need to be tuned depending on the complexity of the image being analyzed. This tuning can take a significant amount of time, so it is prudent to stick to a similar class of simple images (e.g. shapes on a solid background) when testing.
- Lag on the network (such as on *intro-robo*) can greatly influence not just the speed of the robot, but also the accuracy of its movements, and this lag often depends on network load. For example, in our project, lag in the LiDAR data could cause the robot to overshoot its alignment with the wall (which can be mitigated to some extent by lowering the proportional control factor), and lag in sending joint positions can cause the robot to miss a point in a drawing (which can be mitigated by adding in delays to allow time for joint positions to be sent).
