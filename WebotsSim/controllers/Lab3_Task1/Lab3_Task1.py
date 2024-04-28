# FAIRIS-Lite/WebotsSim/controllers/Lab3_Task1/Lab3_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()



# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab3/Lab3_Task1.xml'
robot.load_environment(maze_file)


robot.motion_to_goal(20, 10, 5)

