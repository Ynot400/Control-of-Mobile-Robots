#WebotsSim/controllers/Lab_2_Controller/Lab_2_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab2/Lab2_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
#robot.teleport_robot(-2.5, 0.2, theta=0)
#robot.teleport_robot(-2.5, 0.7, theta=math.pi/8)
#robot.teleport_robot(2.9,0.5,theta=0)

# Main Control Loop for Robot

robot.pid_wall_stop("left", .5, .5, 15, 10)
#robot.pid_wall_stop("right", .5, .5, 8, 6)
#robot.pid_wall_stop("left", .5, .5, 9, 7)





