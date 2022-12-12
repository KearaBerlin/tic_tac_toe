from tic_tac_toe_robot import *
import math

def main():
    ex = TicTacToeRobot()

    SQUARE_WIDTH = 0.03
    orthog_dist = 1/math.sqrt(2) * SQUARE_WIDTH

    CORNER_POSE = [0.43 - 3*orthog_dist, -0.35, 0.07, 90, 0, 90]
    dz = 0.03

    # def mark_guide(nx, ny):
    
    for nx in range(4):
        for ny in range(4):
            [x, y, z , tx, ty, tz] = CORNER_POSE
            x += ny * orthog_dist
            y += ny * orthog_dist
            ex.go_to_pose([x, y, z, tx, ty, tz])

            # move to pose
            print("moving to pose")
            ex.go_to_pose([x, y, z, tx, ty, tz])
            
            # move down by dz to touch paper
            print("moving down")
            ex.go_to_pose([x, y, z-dz, tx, ty, tz])
            
            # move back up
            print("moving back up")
            ex.go_to_pose([x, y, z, tx, ty, tz])

            # ex.go_to_pose(TicTacToeRobot.RESTING_POSE)

            print(f"nx = {nx}, ny = {ny} done")
        
        CORNER_POSE[0] += orthog_dist
        CORNER_POSE[1] -= orthog_dist

    ex.go_to_pose(TicTacToeRobot.RESTING_POSE)        
    print("finished arm guides")

if __name__ == "__main__":
    main()