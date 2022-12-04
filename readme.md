# Tic Tac Toe Robot
This package works with the Kinova Gen3 Lite manipulator arm to play Tic Tac Toe on paper against a human opponent.

## Playing Tic Tac Toe With the Robot
1. Ensure you have python (https://www.python.org/downloads/) and ROS (http://wiki.ros.org/ROS/Installation) installed.
2. Clone this repo into your catkin_ws/src directory.
3. Set up and connect to the Kinova Gen3 Lite robot with your computer (see the manuals at https://www.kinovarobotics.com/product/gen3-lite-robots#Product__resources)
4. Place a marker in the robot's gripper perpendicular to the plane the gripper moves in. For example, you may want to use a dot marker with a large, round tip. Set up your workspace to avoid getting ink on anything accidentally that you don't want to get ink on. You may want to try with the cap on the marker at first.
5. Run in the command line `rosrun tic_tac_toe example_cartesian_poses_with_notifications.py`
6. (Computer vision module to play Tic Tac Toe has not yet been implemented.)

## Files
**example_cartesian_poses_with_notifications.py** is a modified version of one of the kortex_examples, but this version allows the user to enter a cartesian pose in the command line, and the robot arm will move to that pose.

**tic_tac_toe_robot.py** Sets up robot and provides methods to mark a square on a tic tac toe board centered in front of the robot: 

    mark_square(n): Marks square n on the board with a dot.
   
**tic_tac_toe_test.py** small test script that calls `mark_square` from `tic_tac_toe_robot.py` in a loop to mark each of the 9 squares.

**next_move.py** Provides a method `get_best_move(board)` which takes a length 9 list of strings representing a Tic Tac Toe board, and returns the index of the best next move and the best expected score for a game played to completion using that move (1 for win, 0 for tie, -1 for loss).

Example: `['X','X','','O','O','','','','']` returns `(2, 1)` because the board looks like this, with X to play next:

```
 X X -
 O O -
 - - -
```
 
The bottom of the file has some test and example code which is executable.

## More Information
This package was written during Fall 2022 as a project for CSCI 5551 (Introduction to Intelligent Robotic Systems) at the University of Minnesota.

Authors: Keara Berlin, A. N., S. T. I.
