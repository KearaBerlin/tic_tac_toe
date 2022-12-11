# next_move.py
# Provides a method get_best_move(board) which takes a length 9 list of strings
# representing a Tic Tac Toe board, and returns the index of the best next move
# and the best expected score for a game played to completion using that move
# (1 for win, 0 for tie, -1 for loss).
# Example: ['X','X','','O','O','','','',''] returns (2, 1)
# because the board looks like this, with X to play next:
# X X -
# O O -
# - - -
# The bottom of the file has some test and example code which is executable.
# Keara Berlin, 4 December 2022

from enum import Enum
import random
import copy

class Result(Enum):
    INCOMPLETE = 0
    X = 'X'
    O = 'O'
    TIE = 3

# given a board represented as a length 9 list of strings, return the number
# of moves that have been taken so far
def moves_taken(board):
    moves = 0
    for s in board:
        if s != '':
            moves += 1
    return moves

# given a list of 3 strings from a game board, return a string (the winning symbol.)
# a symbol wins if all three elements in the list equal that symbol ("X" or "O")
# if no winner, returns ''
def get_winner(line):
    if line[0] == line[1] and line[1] == line[2]:
        if line[0] == 'X':
            return 'X'
        elif line[0] == 'O':
            return 'O'
    return ''

# given a board (length 9 list of strings), returns the Result
def get_result(board):
    moves = moves_taken(board)
    if moves < 5:
        return Result.INCOMPLETE

    # create list of lines to check for a win (rows, cols, diagonals)
    lines = [board[0:3], board[3:6], board[6:9],
            [board[0], board[4], board[8]],
            [board[2], board[4], board[6]]]
    for list in [[board[j] for j in [i,i+3,i+6]] for i in range(3)]:
        lines.append(list)

    for line in lines:
        winner = get_winner(line)
        if winner == 'X':
            return Result.X
        elif winner == 'O':
            return Result.O

    # check for a tie
    if moves == 9:
        return Result.TIE

    # if it's not a win or a tie, the game is still going
    return Result.INCOMPLETE

# given a board (length 9 list of strings), returns a list of possible next moves (indexes)
def get_possible_moves(board):
    return [i for i in range(len(board)) if board[i] == '']

# given a board (length 9 list of strings), returns either 'X' or 'O' to indicate
# which player moves next.
def get_symbol(board):
    symbol = 'O'
    if moves_taken(board) % 2 == 0:
        symbol = 'X'
    return symbol

# given a board (length 9 list of strings) and an index represting the next move,
# return a board where that move has been taken.
def take_move(board, i):
    new_board = copy.deepcopy(board)
    new_board[i] = get_symbol(board)
    return new_board

# given a board (length 9 list of strings), return the optimal next move (index) and
# its score (1 for a win, 0 for a tie, -1 for a loss) as a tuple.
def get_best_move(board, depth=1):
    symbol = get_symbol(board)
    opponent = 'X'
    if symbol == 'X':
        opponent = 'O'

    print(f'GET BEST MOVE for')
    print_board(board)
    print(f'For player {symbol}')

    moves = get_possible_moves(board)
    #print(f'Moves: {moves}')
    moves_and_scores = []

    # see if any move is a win right away. If so, return that move.
    for move in moves:
        new_board = take_move(board, move)
        result = get_result(new_board)
        if result.name == symbol:
            #print("Returning immediate win move:")
            #print_board(new_board)
            return (move, 10-depth)

    # If there are no immediate wins, score all moves.
    for move in moves:
        new_board = take_move(board, move)
        result = get_result(new_board)
        #print_board(new_board)
        #print(f'result: {result}')

        # find the score
        if result == Result.TIE:
            score = 0
        elif result.name == symbol:
            score = 10 - depth
        else:
            # recurse
            opponent_move, opponent_score = get_best_move(new_board, depth+1)
            score = -1 * opponent_score

        # # either return if it's a win, or store the move and score
        # if score == 1:
        #     #print(f"Returning win move at location {move}")
        #     return (move, score)

        moves_and_scores.append((move,score))

    # pick the best move
    random.shuffle(moves_and_scores)
    moves_and_scores.sort(key = lambda y: y[1])
    print(f'Moves and scores: {moves_and_scores}')

    return moves_and_scores[-1]

# print a length 9 list as 3 rows of 3
def print_board(board):
    print(f'{board[0:3]}\n{board[3:6]}\n{board[6:9]}\n')

# ----------------
# test the moves_taken function
def test_moves_taken(board, expected):
    moves = moves_taken(board)
    if moves != expected:
        print(f"Returned {moves} instead of {expected} for ")
        print_board(board)

# test_moves_taken(['X','X','','O','O','','','',''], 4)
# test_moves_taken(['X','O','X','O','X','O','X','',''], 7)
# test_moves_taken(['X','O','X','O','X','O','X','O',''], 8)
# test_moves_taken(['X','O','X','X','X','O','O','X','O'], 9)
# test_moves_taken(['X','O','','X','','O','X','',''], 5)
# test_moves_taken(['','','','','O','','','',''], 1)
# test_moves_taken(['','','','','','','','',''], 0)

# ----------------
# test the get_result() function
def test_get_result(board, expected):
    result = get_result(board)
    if result != expected:
        print(f"Returned {result} instead of {expected} for board:")
        print_board(board)

# test_get_result(['X','X','','O','O','','','',''], Result.INCOMPLETE)
# test_get_result(['X','O','X','O','X','O','X','',''], Result.X)
# test_get_result(['X','O','X','O','X','O','X','O',''], Result.X)
# test_get_result(['X','O','X','X','X','O','O','X','O'], Result.TIE)
# test_get_result(['X','O','X','X','O','O','X','',''], Result.X)

# -------------------
# test getting rows and columns
# A B C
# d e f
# 7 8 9
# print("Testing rows and columns code")
# board = ['A','B','C','d','e','f','7','8','9']
# print(board[0:3])
# print(board[3:6])
# print(board[6:9])
# print([[board[j] for j in [i,i+3,i+6]] for i in range(3)])
# print([board[0], board[4], board[8]])
# print([board[2], board[4], board[6]])
# lines = [board[0:3], board[3:6], board[6:9],
#         [board[0], board[4], board[8]],
#         [board[2], board[4], board[6]]]
# for list in [[board[j] for j in [i,i+3,i+6]] for i in range(3)]:
#     lines.append(list)
# print(lines)

# ------------------
# examples / testing get_best_move() function
# ------------------
# board = ['' for i in range(9)]
# board = ['X','X','','O','O','','','','']
# board = ['O','O','','X','X','','','','']
# board = ['X','O','X','O','X','O','','','']
# board = ['X','O','X','O','X','X','','O','O']
board = ['X','','X','O','O','O','X','','']
a = get_best_move(board)

i, score = get_best_move(board)
print("\n\n----------------------\n The best move for board: ")
print_board(board)
print(f"Is at index {i}")
