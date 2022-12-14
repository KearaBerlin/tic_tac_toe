#!/usr/bin/env python3
###
# Play one move of Tic Tac Toe.
###

from tic_tac_toe_robot import TicTacToeRobot as Robot
from tic_tac_toe_CV import *
from next_move import get_best_move
# from tic_tac_toe_CV import get_image, create_game_state
import sys

def main():
    robot = Robot()
    
    image = get_image()
    board = create_game_state(image)
    (square, score) = get_best_move(board)
    
    robot.mark_square(square)
    
    print(board, square)

if __name__ == "__main__":
    main()