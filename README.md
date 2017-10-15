# TurtleBot_MazeSolver
TurtleBot solves a maze without cheating (hopefully)

TEAM JRM'S MAZE SOLVER, by Rosalind Ellis, Melanie Kaplan, Jonas Tjahjadi

TASK: To learn the basics of the ROS structure by implementing a maze solver in Python. Degrees of abstraction have been implemented,
but not to a full extent.
PROBLEMS ENCOUNTERED: how to get STDR to work (hardest problem, I posted on ROS about it and I got roasted, important to understand
how to create a map and make your own robot), connecting to the turtlebot, using VMWare or VirtualBox, 
getting the right packages and setup for EACH person on the team

IMPLEMENTATION: wall follow algorithm on the left wall.


Languages / Requirements: STDR, setting the yaml file up, understanding how to customize a robot in STDR.


NOTES and THOUGHTS:
For proper usage, pay attention to the parameters in which the robot follows the wall. There's a threshold that you should obey otherwise
the robot could break. This is very important for STDR .yaml file, as a gigantic space will take 40 minutes to solve the maze unless the
speeds are changed (as we have observed in class). If it's too small, then the robot would end up crashing to a wall. Another observation
that the whole class has seen in our first implementation is that it would just go too fast or wouldn't turn quick enough and just crash
into a wall. This would result in the laser sensor not being able to understand what it's seeing.

Turtlebot could be run on both the simulator and the turtlebot. Specify the arg when running the MazeSolver node.

Observe also that we have specified speeds to send to cmd_vel. These speeds are always subject to change, so if you use this code in the
future, you may deal with problems with actual hardware. You may deal with outside real-life factors and it's totally fine. For the future
though, we should understand how to use PID to adjust to many surfaces and not have to tweak it based on the environment.

SUGGESTIONS FOR FUTURE:
This project should be done within the first month of the semester. Students should not dwindle on Arduino for too long, they should focus
on ROS for the majority of the semester.
