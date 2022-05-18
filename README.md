# Robo Forger

## WRITEUP:

### Project Description
Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate.

### System Architecture
Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components.

### Execution
Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.

### Challenges, Future Work, & Takeaways
These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways).


## Running:

- Make sure you're running `roscore`
- Launch the robot's manipulation stuff
  - `roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch`
- Launch the controller for the arm
  - `roslaunch turtlebot3_manipulation_moveit_config move_group.launch`
