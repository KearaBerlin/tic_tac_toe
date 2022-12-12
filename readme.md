# Tic Tac Toe Robot
This package works with the Kinova Gen3 Lite manipulator arm to play Tic Tac Toe on paper against a human opponent.

## Playing Tic Tac Toe With the Robot
1. Ensure you have installed python (https://www.python.org/downloads/), ROS (http://wiki.ros.org/ROS/Installation), and the ROS Kortex API (https://github.com/Kinovarobotics/ros_kortex).
2. Clone this repo into your catkin_ws/src directory.
3. Set up and connect to the Kinova Gen3 Lite robot with your computer (see the manuals at https://www.kinovarobotics.com/product/gen3-lite-robots#Product__resources)
4. Set up a camera looking down on only the tic tac toe board.
5. Use the Web App to set protection zones around the camera.
6. Place a marker in the robot's gripper perpendicular to the plane the gripper moves in. For example, you may want to use a dot marker with a large, round tip. Set up your workspace to avoid getting ink on anything accidentally that you don't want to get ink on. You may want to try with the cap on the marker at first. Place a tic tac toe board under the camera with the center at x,y relative to the robot base (0.4, -0.4) and with the top of the board pointing straight away from the robot base.
7. In a command line window, run `roscore`
8. In another window, run `roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite`
9. Run in another window `roslaunch tic_tac_toe cartesian_poses_with_notifications_python.launch` which will prompt you to enter a Cartesian pose for the robot to go to. You may enter a pose anytime or simply leave the prompt up while the robot is in use.
10. Run in another window `python3 play.py` which will look at the board, determine the game state, find the optimal next move, and mark the board.
11. Make your own next move in the game and repeat step 10.

## Files
**example_cartesian_poses_with_notifications.py** is a modified version of one of the kortex_examples, but this version allows the user to enter a cartesian pose in the command line, and the robot arm will move to that pose.

**tic_tac_toe_robot.py** Sets up robot and provides methods to mark a square on a tic tac toe board centered in front of the robot, as well as to draw a line between two squares on the tic tac toe board: 

    mark_square(n): Marks square n on the board with a dot.
    draw_line(s, e): Draws a line on the board between squares s and e.
   
**tic_tac_toe_test.py** small test script that calls `mark_square` from `tic_tac_toe_robot.py` in a loop to mark each of the 9 squares.

**next_move.py** Provides a method `get_best_move(board)` which takes a length 9 list of strings representing a Tic Tac Toe board, and returns the index of the best next move and the best expected score for a game played to completion using that move (1 for win, 0 for tie, -1 for loss).

Example: `['X','X','','O','O','','','','']` returns `(2, 1)` because the board looks like this, with X to play next:

```
 X X -
 O O -
 - - -
```
 
The bottom of the file has some test and example code which is executable.

**tic_tac_toe_CV.py** provides methods to get an image from a USB camera and to generate the current board state from an image:

    get_image(): Returns a still image from the camera attached to the computer (you may need to change the device name in VideoCapture("/dev/video0"))
    create_game_state(image): Returns a length 9 list of strings representing the game board state seen in image.

**play.py** Plays one move of Tic Tac Toe autonomously.

## More Information
This package was written during Fall 2022 as a project for CSCI 5551 (Introduction to Intelligent Robotic Systems) at the University of Minnesota.

Authors: Keara Berlin, Amarachi Nzeukwu, Sai Tarun Inaganti
