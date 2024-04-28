#WebotsSim/controllers/Lab2_Task2_Right/Lab2_Task2_Right.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()



# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab2/Lab2_EC_2.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()


# EC Maze 1 solution
# robot.move_forward(1,10)
# robot.rotate(90,2,"left")
# robot.pid_around_wall(15,15,.48,"right")

# EC Maze 2 solution
robot.move_forward(1,10)
robot.rotate(90,2,"left")
robot.pid_around_wall(20,15,.4,"right")