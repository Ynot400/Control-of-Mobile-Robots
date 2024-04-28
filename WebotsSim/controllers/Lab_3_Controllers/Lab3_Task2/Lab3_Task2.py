# FAIRIS-Lite/WebotsSim/controllers/Lab3_Task2/Lab3_Task2.py
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



robot.teleport_robot(-4.5,0)
robot.rotate(-50,2)
robot.bug_0(18, 7, 5, -90, .5, 15)