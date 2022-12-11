from tic_tac_toe_robot import *
import math

def main():
    ex = TicTacToeRobot()

    SQUARE_WIDTH = 0.03
    SQUARE_WIDTH *= 1/math.sqrt(2)

    CORNER_POSE = [0.43 - 1.5*SQUARE_WIDTH, -0.35 - 1.5*SQUARE_WIDTH, 0.07, 90, 0, 90]
    dz = 0.03

    # def mark_guide(nx, ny):
    
    for nx in range(4):
        for ny in range(4):
            [x, y, z , tx, ty, tz] = CORNER_POSE
            x += nx * x
            y += ny * y
            self.go_to_pose([x, y, z, tx, ty, tz])

            # move to pose
            print("moving to pose")
            self.go_to_pose([x, y, z, tx, ty, tz])
            
            # move down by dz to touch paper
            print("moving down")
            self.go_to_pose([x, y, z-dz, tx, ty, tz])
            
            # move back up
            print("moving back up")
            self.go_to_pose([x, y, z, tx, ty, tz])

            print(f"nx = {nx}, ny = {ny} done")
            
    print("finished arm guides")

if __name__ == "__main__":
    main()