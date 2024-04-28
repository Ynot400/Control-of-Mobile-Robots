# WebotsSim/controllers/lab4_Task1/lab4_Task1.py
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot(0,0)



# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab4/Lab4_Task1_1.xml'
robot.load_environment(maze_file)
robot.move_to_start()
robot.landmark_localization_with_trilateration()
