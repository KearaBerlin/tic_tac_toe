#!/usr/bin/env python3
###
# Test calling the method in tic_tac_toe_robot.py
###

from tic_tac_toe_robot import TicTacToeRobot as Robot
import sys

if __name__ == "__main__":
    robot = Robot()
    if len(sys.argv) >= 2:
        i = int(sys.argv[1])
        if 0 <= i and i <= 8:
            robot.mark_square(i)
    else:
        for i in range(9):
    	    robot.mark_square(i)
