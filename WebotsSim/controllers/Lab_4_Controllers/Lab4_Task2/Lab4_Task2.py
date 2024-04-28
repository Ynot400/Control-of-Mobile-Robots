# WebotsSim/controllers/Lab4_Task2/Lab4_Task2.py
import os
import random
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot(0,0)



# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab4/Lab4_Task2_2.xml'
robot.load_environment(maze_file)
robot.move_to_start()


maze_cells = [(0, 0), (0, 1), (0, 2), (0, 3),
              (1, 0), (1, 1), (1, 2), (1, 3),
              (2, 0), (2, 1), (2, 2), (2, 3),
              (3, 0), (3, 1), (3, 2), (3, 3)]

track_visited_cells = ['.', '.', '.', '.',
'.', '.', '.', '.',
'.', '.', '.', '.',
'.', '.', '.', '.']


# the following maze and cell code was taken as a resource from canvas files under lab 4

class Cell:
	def __init__(self, west, north, east, south, visited = False):
		# There are 4 walls per cell
		# Wall values can be 'W', 'O', or '?' (wall, open, or unknown)
		self.west = west
		self.north = north
		self.east = east
		self.south = south
		
		# Store whether or not the cell has been visited before
		self.visited = visited

		


# maze for task 2 map 1
# maze = [
	# Cell('W','W','O','O', False), Cell('O','W','O','W', False), Cell('O','W','O','W', False), Cell('O','W','W','O', False),
	# Cell('W','O','W','O', False), Cell('W','W','O','O', False), Cell('O','W','W','O', False), Cell('W','O','W','O', False),
	# Cell('W','O','W','O', False), Cell('W','O','O','W', False), Cell('O','O','O','W', False), Cell('O','O','W','W', False),
	# Cell('W','O','O','W', False), Cell('O','W','O','W', False), Cell('O','W','O','W', False), Cell('O','W','W','W', False)
# ]
        
# maze for task 2 map 2	
maze = [
	Cell('W','W','O','W', False), Cell('O','W','O','W', False), Cell('O','W','O','O', False), Cell('O','W','W','O', False),
	Cell('W','W','O','O', False), Cell('O','W','O','O', False), Cell('O','O','W','O', False), Cell('W','O','W','O', False),
	Cell('W','O','W','O', False), Cell('W','O','O','W', False), Cell('O','O','W','W', False), Cell('W','O','W','O', False),
	Cell('W','O','O','W', False), Cell('O','W','O','W', False), Cell('O','W','O','W', False), Cell('O','O','W','W', False)
]	



starting_x, starting_y = robot.find_starting_position() # find the starting position of the robot
robot.set_pose(starting_x, starting_y)
# for loop grabs the row ranges, row will update to the index of the row
row_ranges = [(1, 2, 0), (0, 1, 1), (-1, 0, 2), (-2, -1, 3)]
for lower, upper, index in row_ranges:
    if lower <= starting_y <= upper:
        row = index
        break

# for loop grabs the column ranges, column will update to the index of the column
column_ranges = [(-2, -1, 0), (-1, 0, 1), (0, 1, 2), (1, 2, 3)]
for lower, upper, index in column_ranges:
    if lower <= starting_x <= upper:
        column = index
        break


pair_of_coordinates = (row, column) # turn the row and column into a pair of coordinates

for i, pair in enumerate(maze_cells): # iterate through the cell coordinates, keeping the index value
    if pair == pair_of_coordinates:
        current_index = i
        break
    else:
        continue
robot.find_probability_for_lab4(current_index, maze) # calls a function that will find the probability of the robot being in all of the cells
print(f"Probability Map based on grid number: {current_index+1} oriented at {robot.get_compass_reading()}")
print("x coordinate:", round(robot.x, 2), "y coordinate:", round(robot.y, 2))


older_distance = sum(robot.get_encoder_readings()) / len(robot.get_encoder_readings()) * robot.wheel_radius # start grabbing the older distance for calulations involving the pose of the robot
while (robot.experiment_supervisor.step(robot.timestep) != -1): # loop through all of the code above until the robot visits all of the cells
    # print("x coordinate:", round(starting_x, 2), "y coordinate:", round(starting_y, 2), "grid number n:", i+1, "Theta:", robot.get_compass_reading())
    if track_visited_cells[current_index] == '.':
        track_visited_cells[current_index] = 'X'
        if all(x == track_visited_cells[0] for x in track_visited_cells): # check if all cells have been visited
            print("All cells have been visited")
            for k in range(0, len(track_visited_cells), 4): # print the visited cells variable in a 4x4 grid
                print(' '.join(track_visited_cells[k:k+4]))
            break
    else:
        print("Already visited this cell")

    for j in range(0, len(track_visited_cells), 4): # print the visited cells variable in a 4x4 grid
        print(' '.join(track_visited_cells[j:j+4]))
    # Algorithm works as follows: front/left/right wall will be 0 if there is no wall is detected, if there is a wall, it will evaluate to 1
    front_Wall = 0 if min(robot.get_lidar_range_image()[390:410]) > 0.7 else 1
    left_Wall = 0 if min(robot.get_lidar_range_image()[190:210]) > 0.7 else 1
    right_Wall = 0 if min(robot.get_lidar_range_image()[590:610]) > 0.7 else 1
    
    # if statements to handle the movement of the robot accordingly
    if (front_Wall == 0):
        robot.move_forward(1,12)
        robot.straighten_that_angle()
        
    elif (left_Wall == 0):
        robot.rotate(90, 2)
        robot.straighten_that_angle()
        # robot.straighten_that_angle()
        robot.move_forward(1,12)
        # robot.straighten_that_angle()

    
    elif (right_Wall == 0):
        robot.rotate(-90, 2)
        robot.straighten_that_angle()
        # robot.straighten_that_angle()
        robot.move_forward(1,12)
        
    else:
        robot.rotate(180, 2)
        robot.straighten_that_angle()
    


    robot.find_probability_for_lab4(current_index, maze) # calls a function that will find the probability of the robot being in all of the cells
    # accumulated_distance0 = 0
    coordinate_pair = robot.find_coordinates_for_lab4(robot.x,robot.y) # find the coordinates of the robot
    for i, pair in enumerate(maze_cells): # iterate through the cell coordinates, keeping the index value
        if pair == coordinate_pair:
            current_index = i
            break
        else:
            continue

    print(f"Probability Map based on grid number: {i+1} oriented at {robot.get_compass_reading()}")
    print("x coordinate:", round(robot.x, 2), "y coordinate:", round(robot.y, 2))
    


    

