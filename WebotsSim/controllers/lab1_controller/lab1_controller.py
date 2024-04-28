# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
robot.move_forward(2.5, 15)
robot.circular_rotation(math.pi/2,.5,10,"right")
robot.rotate(90, 1, "left")
robot.move_forward(2, 15)
robot.circular_rotation(math.pi/2,.5,10,"right")
robot.rotate(0, 0.5, "left")
robot.move_forward(2,15)
robot.circular_rotation(math.pi/2,.5,10,"right")
robot.rotate(270, 1, "left")
robot.move_forward(1,15)
robot.circular_rotation(math.pi,.75,10,"right")
robot.rotate(90,2,"left")

