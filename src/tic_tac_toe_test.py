#!/usr/bin/env python3
###
# Test calling the method in tic_tac_toe_robot.py
###

from tic_tac_toe_robot import TicTacToeRobot as Robot

if __name__ == "__main__":
    robot = Robot()
    for i in range(9):
    	robot.mark_square(i)
