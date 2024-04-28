# Relative Path: FAIRIS-Lite/WebotsSim/libraries/MyRobot.py

from WebotsSim.libraries.RobotLib.RosBot import RosBot 
import math
import random
import numpy as np
from statistics import mean 
class MyRobot(RosBot):

   def __init__(self, x, y):
        RosBot.__init__(self)
        self.x = x
        self.y = y
   
   def set_pose(self, x, y): # call this function to set the robots pose
      self.x = x
      self.y = y
      return
   
   def adjust_sensor_readings(self, front_sensor, left_sensor, right_sensor, back_sensor):
    # Define sensor mapping relative to orientation
    orientation_mapping = {
        "N": {"front": front_sensor, "left": left_sensor, "right": right_sensor},
        "E": {"front": left_sensor, "left": back_sensor, "right": front_sensor},
        "S": {"front": back_sensor, "left": right_sensor, "right": left_sensor},
        "W": {"front": right_sensor, "left": front_sensor, "right": back_sensor}
    }
    orientation = self.get_compass_reading()
    if orientation in range(80, 100):
         robot_orientation = "N"
    elif orientation in range(170, 190):
         robot_orientation = "W"
    elif orientation in range(260, 280):
         robot_orientation = "S"
    elif orientation in range(350, 361) or orientation in range(0, 11):
         robot_orientation = "E"
       
    # Get sensor readings based on robot's orientation
    orientation_sensors = orientation_mapping[robot_orientation]
    
    # Extract adjusted sensor readings
    adjusted_front_sensor = orientation_sensors["front"]
    adjusted_left_sensor = orientation_sensors["left"]
    adjusted_right_sensor = orientation_sensors["right"]
    
    return adjusted_front_sensor, adjusted_left_sensor, adjusted_right_sensor


   def find_coordinates_for_lab4(self, x, y):
      row_ranges = [(1, 2, 0), (0, 1, 1), (-1, 0, 2), (-2, -1, 3)]
      for lower, upper, index in row_ranges:
         if lower <= y <= upper:
            row = index
            break

      # for loop grabs the column ranges, column will update to the index of the column
      column_ranges = [(-2, -1, 0), (-1, 0, 1), (0, 1, 2), (1, 2, 3)]
      for lower, upper, index in column_ranges:
         if lower <= x <= upper:
            column = index
            break

      return (row, column)

   def straighten_that_angle(self):
      # reset the robot's position so that it stays straight and doesnt have incorrect angles
      if self.get_compass_reading() in range(5, 95):  
         print("Reshifting to 90 degrees")
         while self.experiment_supervisor.step(self.timestep) != -1:
            if self.get_compass_reading() not in range(89, 91):
               self.set_left_motors_velocity(-2)
               self.set_right_motors_velocity(2)
            else:
               print("Reshifted to 90 degrees:", self.get_compass_reading())
               return

      elif self.get_compass_reading() in range(96, 186):
         print("Reshifting to 180 degrees")
         while self.experiment_supervisor.step(self.timestep) != -1:
            if self.get_compass_reading() not in range(179, 181):
               self.set_left_motors_velocity(-2)
               self.set_right_motors_velocity(2)
            else:
               print("Reshifted to 180 degrees:", self.get_compass_reading())
               return
      elif self.get_compass_reading() in range(187, 276):
         print("Reshifting to 270 degrees")
         while self.experiment_supervisor.step(self.timestep) != -1:
            if self.get_compass_reading() not in range(269, 271):
               self.set_left_motors_velocity(-2)
               self.set_right_motors_velocity(2)
            else:
               print("Reshifted to 270 degrees:", self.get_compass_reading())
               return

      elif self.get_compass_reading() in range(277, 4):
         print("Reshifting to 360 degrees")
         while self.experiment_supervisor.step(self.timestep) != -1:
               if self.get_compass_reading() not in range(357, 359):
                  self.set_left_motors_velocity(-2)
                  self.set_right_motors_velocity(2)
               else:
                  print("Reshifed to 360 degrees:", self.get_compass_reading())
                  return


   def find_starting_position(self):
      start_pos = self.starting_position
      starting_x = start_pos.x
      starting_y = start_pos.y
      # theta = start_pos.theta
      # print(starting_x,starting_y,theta)
      return starting_x, starting_y




   def find_probability_for_lab4(self, index, maze):
    probability_map =[0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0]
   
    adjusted_front_sensor, adjusted_left_sensor, adjusted_right_sensor = self.adjust_sensor_readings(self.get_lidar_range_image()[400], self.get_lidar_range_image()[200], self.get_lidar_range_image()[600], self.get_lidar_range_image()[0])

    z_left = 1 if adjusted_left_sensor < 0.8 else 0
    z_right = 1 if adjusted_right_sensor < 0.8 else 0
    z_front = 1 if adjusted_front_sensor < 0.8 else 0

  
    for i, cell in enumerate(maze):
      # Grab S values for each cell
      next_west = 1 if cell.west == "W" else 0
      next_north = 1 if cell.north == "W" else 0
      next_east = 1 if cell.east == "W" else 0
   
      cell_pb = self.probability_for_lab4(next_west, z_left) * self.probability_for_lab4(next_north, z_front) * self.probability_for_lab4(next_east, z_right)

      probability_map[i] = cell_pb
   
    norm_factor = 1/sum(probability_map)

    for i, prob in enumerate(probability_map):
      probability_map[i] = norm_factor * prob
   
    probability_map = [round(prob, 3) for prob in probability_map] # round the probabilities to 3 decimal places
   
    for i in range(0, len(probability_map), 4): # print the visited cells variable in a 4x4 grid
      print(' '.join(map(str, probability_map[i:i+4])))
      
      
    return

   def probability_for_lab4(self, S, Z):

    prob = 0

    if S == 0:
        if Z == 0:
            prob = 0.7
        else:
            prob = 0.3

    else:
        if Z == 1:
            prob = 0.9
        else:
            prob = 0.1

    return prob

   def update_robot_pose(self, older_distance):

      # Obtain encoder readings
      encoder_readings = self.get_encoder_readings()
      # Calculate distance moved by each wheel
      distance_average = ((sum(encoder_readings) / 4.0) * self.wheel_radius) - older_distance
      # Obtain compass bearing
      bearing = self.get_compass_reading()
      # Calculate new pose based on distance moved and bearing
      # This will require additional calculations depending on your implementation
      # Update robot's internal pose representation here
      self.x = (self.x + distance_average * math.cos(math.radians(bearing))) 
      self.y = (self.y + distance_average * math.sin(math.radians(bearing))) 
      return



   def wall_localization(self):
      cell_coordinates = [(0, 0), (0, 1), (0, 2), (0, 3),
              (1, 0), (1, 1), (1, 2), (1, 3),
              (2, 0), (2, 1), (2, 2), (2, 3),
              (3, 0), (3, 1), (3, 2), (3, 3)]
      
      track_visited_cells = ['.', '.', '.', '.',
            '.', '.', '.', '.',
            '.', '.', '.', '.',
            '.', '.', '.', '.']
   


   def rotate_to_this_degree(self, degree, vel):
      while self.experiment_supervisor.step(self.timestep) != -1:
        if degree > 0:
            self.set_rear_left_motor_velocity(velocity=-vel)
            self.set_rear_right_motor_velocity(velocity=vel)
            self.set_front_left_motor_velocity(velocity=-vel)
            self.set_front_right_motor_velocity(velocity=vel)
        else:
            self.set_rear_left_motor_velocity(velocity=vel)
            self.set_rear_right_motor_velocity(velocity=-vel)
            self.set_front_left_motor_velocity(velocity=vel)
            self.set_front_right_motor_velocity(velocity=-vel)
   
        if degree - 2 <= self.get_compass_reading() <= degree + 2:
            self.stop()
            return

   def find_empty_cell(self, current_index, track_visited_cells):
      task_complete = False
      # The breakdown of the algorithm is as follows:
      # 1. Find the empty cell index
      # 2. Check if the empty cell index is greater than the current index and the absolute difference between the two is less than or equal to 3, if so, if the current position and the current empty spot is on the same row, move left until you encounter it, if it is a special case where the empty cell is below, move down
      # 3. Check if the empty cell index is less than the current index and the absolute difference between the two is less than or equal to 3, if so, if the current position and the current empty spot is on the same row, move right until you encounter it, if it is a special case where the empty cell is above, move up
      # 4. If the empty cell index is greater than the current index and the absolute difference between the two is greater than 3, move down
      # 5. If the empty cell index is less than the current index and the absolute difference between the two is greater than 3, move up
      empty_cell_index = None

      for index, cell in enumerate(track_visited_cells):
        if cell == '.':
            empty_cell_index = index
            break
      if all(x == track_visited_cells[0] for x in track_visited_cells):
       return   
         
      # print("Empty Cell Index:", empty_cell_index)
      # print("Current Index:", current_index)
      # print("absolute difference:", abs(empty_cell_index - current_index))
      if empty_cell_index > current_index and abs(empty_cell_index - current_index) <= 3:
         if current_index in range (0, 4) and empty_cell_index in range(0, 4):
               self.rotate_to_this_degree(0, 4)
         elif current_index in range(4, 8) and empty_cell_index in range(4, 8):
               self.rotate_to_this_degree(0, 4)
         elif current_index in range(8, 12) and empty_cell_index in range(8, 12):
               self.rotate_to_this_degree(0, 4)
         elif current_index in range(12, 16) and empty_cell_index in range(12, 16):
               self.rotate_to_this_degree(0, 4)
         else:
             self.rotate_to_this_degree(270,4)
      elif empty_cell_index < current_index and abs(empty_cell_index - current_index) <= 3:
            # print("go left")
            if current_index in range (0, 4) and empty_cell_index in range(0, 4):
               # print("should work")
               self.rotate_to_this_degree(180, 4)
            elif current_index in range(4, 8) and empty_cell_index in range(4, 8):
               self.rotate_to_this_degree(180, 4)
            elif current_index in range(8, 12) and empty_cell_index in range(8, 12):
               self.rotate_to_this_degree(180, 4)
            elif current_index in range(12, 16) and empty_cell_index in range(12, 16):
               self.rotate_to_this_degree(180, 4)
            else:
               self.rotate_to_this_degree(90, 4)
      elif empty_cell_index > current_index and abs(empty_cell_index - current_index) > 3:
         # print("go down")
         self.rotate_to_this_degree(270,4)
      else:
         # print("go up")
         self.rotate_to_this_degree(90, 4)
      return
         
         
             



   def trilateration(self, red, green, yellow):
      yellow_x1, green_x3 = -2, -2
      red_x2 = 2
      yellow_y1, red_y2 = 2, 2
      green_y3 = -2
      # formula derived from the slides in class
      A = 2 * (-yellow_x1 + red_x2)
      # print("A:", A)
      B = 2 * (-yellow_y1 + red_y2)
      # print("B:", B)
      C = (yellow ** 2) - (red ** 2) - (yellow_x1 ** 2) + (red_x2 ** 2) - (yellow_y1 ** 2) + (red_y2 ** 2)
      # print("C:", C)

      D = 2 * (-red_x2 + green_x3)
      # print("D:", D)
      E = 2 * (-red_y2 + green_y3)
      # print("E:", E)
      F = (red ** 2) - (green ** 2) - (red_x2 ** 2) + (green_x3 ** 2) - (red_y2 ** 2) + (green_y3 ** 2)
      # print("F:", F)

      x = ((C * E) - (F * B)) / ((E * A) - (B * D))
      y = ((C * D) - (A * F)) / ((B * D) - (A * E))

      # print("x:", x)
      # print("y:", y)

      return x, y




   def landmark_localization_with_trilateration(self):
       cylinder_radius = 0.314
       yellow_radius, green_radius, red_radius = 0, 0, 0
       check1, check2, check3 = 0, 0, 0
       row = None
       column = None

       track_visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.']

       cell_coordinates = [(0, 0), (0, 1), (0, 2), (0, 3),
                (1, 0), (1, 1), (1, 2), (1, 3),
                (2, 0), (2, 1), (2, 2), (2, 3),
                (3, 0), (3, 1), (3, 2), (3, 3)]
      

   
       while self.experiment_supervisor.step(self.timestep) != -1:
         objects = self.rgb_camera.getRecognitionObjects()
         self.set_left_motors_velocity(-4)
         self.set_right_motors_velocity(4)
         # obtain distance values for three objects, the fourth one (blue in our case) can be ignored
         for obj in objects:
             if obj.getColors()[0] == 1 and obj.getColors()[1] == 1:
               # print("Yellow detected")
               yellow_radius = obj.getPosition()[0] + cylinder_radius
               check1 = 1

             elif obj.getColors()[1] == 1 and obj.getColors()[0] != 1:
               # print("Green detected")
               green_radius = obj.getPosition()[0] + cylinder_radius
               check2 = 1

             elif obj.getColors()[0] == 1:
               # print("Red detected")
               red_radius = obj.getPosition()[0] + cylinder_radius
               check3 = 1

             elif obj.getColors()[2] == 1:
               continue
               # print("Blue Cylinder Ignored")
         

           
             if check1 == 1 and check2 == 1 and check3 == 1:
               print("All objects have been detected")
               # reset the robot's position so that it stays straight and doesnt have incorrect angles
               if self.get_compass_reading() in range(0, 90):  
                  print("Reshifting to 90 degrees")
                  while self.experiment_supervisor.step(self.timestep) != -1:
                     if self.get_compass_reading() not in range(89, 91):
                        self.set_left_motors_velocity(-3)
                        self.set_right_motors_velocity(3)
                     else:
                        print("Reshifted to 90 degrees:", self.get_compass_reading())
                        break

               elif self.get_compass_reading() in range(91, 180):
                  print("Reshifting to 180 degrees")
                  while self.experiment_supervisor.step(self.timestep) != -1:
                     if self.get_compass_reading() not in range(179, 181):
                        self.set_left_motors_velocity(-3)
                        self.set_right_motors_velocity(3)
                     else:
                        print("Reshifted to 180 degrees:", self.get_compass_reading())
                        break

               elif self.get_compass_reading() in range(181, 270):
                  print("Reshifting to 270 degrees")
                  while self.experiment_supervisor.step(self.timestep) != -1:
                     if self.get_compass_reading() not in range(269, 271):
                        self.set_left_motors_velocity(-3)
                        self.set_right_motors_velocity(3)
                     else:
                        print("Reshifted to 270 degrees:", self.get_compass_reading())
                        break

               elif self.get_compass_reading() in range(271, 359):
                  print("Reshifting to 360 degrees")
                  while self.experiment_supervisor.step(self.timestep) != -1:
                        if self.get_compass_reading() not in range(357, 359):
                           self.set_left_motors_velocity(-3)
                           self.set_right_motors_velocity(3)
                        else:
                           print("Reshifed to 360 degrees:", self.get_compass_reading())
                           break
               # print("radius of yellow, green, red:", yellow_radius, green_radius, red_radius)
               x, y = self.trilateration(red_radius, green_radius, yellow_radius)
               # print("x:", x, "y:", y)


               # for loop grabs the row ranges, row will update to the index of the row
               row_ranges = [(1, 2, 0), (0, 1, 1), (-1, 0, 2), (-2, -1, 3)]
               for lower, upper, index in row_ranges:
                  if lower <= y <= upper:
                        row = index
                        break

               # for loop grabs the column ranges, column will update to the index of the column
               column_ranges = [(-2, -1, 0), (-1, 0, 1), (0, 1, 2), (1, 2, 3)]
               for lower, upper, index in column_ranges:
                  if lower <= x <= upper:
                        column = index
                        break
               
               pair_of_coordinates = (row, column) # turn the row and column into a pair of coordinates
               for i, pair in enumerate(cell_coordinates): # iterate through the cell coordinates, keeping the index value
                  if pair == pair_of_coordinates:
                     current_index = i
                     break
                  else:
                     continue
               
               # print("Pair of Coordinates:", pair_of_coordinates)
               # print("Index of Pair of Coordinates:", i)
               if track_visited_cells[current_index] == '.':
                  track_visited_cells[current_index] = 'X'
                  for i in range(0, len(track_visited_cells), 4): # print the visited cells variable in a 4x4 grid
                     print(' '.join(track_visited_cells[i:i+4]))
               else:
                  print("Cell has already been visited")
                  self.find_empty_cell(current_index, track_visited_cells)



               if all(x == track_visited_cells[0] for x in track_visited_cells): # check if all cells have been visited
                  print("All cells have been visited")
                  for k in range(0, len(track_visited_cells), 4): # print the visited cells variable in a 4x4 grid
                     print(' '.join(track_visited_cells[k:k+4]))
                  return

                  
         
               print("x coordinate:", round(x, 2), "y coordinate:", round(y, 2), "grid number n:", current_index+1, "Theta:", self.get_compass_reading())
               
            
               # print("should print after cell shown or cell has been visited")
               self.move_forward(1, 10)


               # reset variables
               yellow_radius, green_radius, red_radius = 0, 0, 0
               check1, check2, check3 = 0, 0, 0




         


  
   
   def bug_0(self, velocity, K_object, K_curve, rotating_degree, distance_from_wall, proportional_gain):
    turnError = 1
    vel = 6
    max_vel = 20
    min_vel = -20
    state = 0
    while self.experiment_supervisor.step(self.timestep) != -1:
        if state == 0:
            objects = self.rgb_camera.getRecognitionObjects()
            if not objects:  # Check if the list is empty
                if (self.get_lidar_range_image()[400] < distance_from_wall):
                    self.rotate(rotating_degree, 5)
                    state = 1
                    continue      
            for obj in objects:
                  position_on_image = obj.getPosition()
                  print(self.get_lidar_range_image()[400])
                  if (min(self.get_lidar_range_image()[500:700]) < .15 or min(self.get_lidar_range_image()[100:300]) < .15) and position_on_image[0] > 1:
                     # print("dont")
                     self.rotate(rotating_degree, 5)
                     state = 1
                     continue

                  error = (-0.5) - (-position_on_image[0])
                  turnError = position_on_image[1]

                  print("Position:", position_on_image[0], position_on_image[1])
                  control = error * K_object
                  if control > max_vel:
                     velocity = max_vel
                  elif min_vel <= control <= max_vel:
                     velocity = control
                  else:
                     velocity = min_vel

                  if turnError > 0:
                     self.pid_turn_left(velocity - (abs(turnError) * K_curve), velocity)
                  else:
                     self.pid_turn_right(velocity - (abs(turnError) * K_curve), velocity)
                  print("control:", control)
                  if ( 0.0 <= control <= 2.0):
                     print("Program should now stop")
                     self.stop()
   
               
                  continue
         # wall following code
        elif state == 1:
            print("forward:", min(self.get_lidar_range_image()[350:450]))
            if (min(self.get_lidar_range_image()[500:700]) < .1 or min(self.get_lidar_range_image()[100:300]) < .1 or min(self.get_lidar_range_image()[350:450]) < .3):
               # print("dont")
               self.rotate(rotating_degree, 5)
               continue
            if rotating_degree == -90:
                  objects = self.rgb_camera.getRecognitionObjects()
                  if objects:
                      state = 0
                      continue
                  average_range_image = min(self.get_lidar_range_image()[180:220])
                  error = (distance_from_wall) - average_range_image
                  control = abs(error) * proportional_gain
                  if self.get_lidar_range_image()[400] < 0.3:
                     self.rotate(rotating_degree, 5)
                     continue
                  elif error > 0:
                        self.pid_turn_right(velocity - control, velocity)
                  else:
                        self.pid_turn_left(velocity - control, velocity)
                  continue
            
            else:
                  objects = self.rgb_camera.getRecognitionObjects()
                  if objects:
                      state = 0
                      continue
                  average_range_image = min(self.get_lidar_range_image()[580:620])
                  error = (distance_from_wall) - average_range_image
                  control = abs(error) * proportional_gain
                  print(average_range_image)
                  print("error:", error)
                  print("control:", control)
   
                  if self.get_lidar_range_image()[400] < 0.3:
                     self.rotate(rotating_degree, 5)
                     continue
                  elif error > 0:
                        self.pid_turn_left(velocity - control, velocity)
                  else:
                        self.pid_turn_right(velocity - control, velocity)
                  continue
         
    
   def motion_to_goal(self, velocity, K_object, K_curve):
    turnError = 1
    vel = 6
    max_vel = 20
    min_vel = -20
    state = 0
    while self.experiment_supervisor.step(self.timestep) != -1:
      objects = self.rgb_camera.getRecognitionObjects()
      if not objects:  # Check if the list is empty
            if turnError > 0:
               self.set_rear_left_motor_velocity(velocity=-vel)
               self.set_rear_right_motor_velocity(velocity=vel)
               self.set_front_left_motor_velocity(velocity=-vel)
               self.set_front_right_motor_velocity(velocity=vel)
            else:
               self.set_rear_left_motor_velocity(velocity=vel)
               self.set_rear_right_motor_velocity(velocity=-vel)
               self.set_front_left_motor_velocity(velocity=vel)
               self.set_front_right_motor_velocity(velocity=-vel)
      else:
            for obj in objects:
                  position_on_image = obj.getPosition()
                  # if not obj:
                  #    break
                  error = (-0.5) - (-position_on_image[0])
                  turnError = position_on_image[1]

                  print("Position:", position_on_image[0], position_on_image[1])
                  control = error * K_object
                  if control > max_vel:
                     velocity = max_vel
                  elif min_vel <= control <= max_vel:
                     velocity = control
                  else:
                     velocity = min_vel
               
                  if turnError > 0:
                     self.pid_turn_left(velocity - (abs(turnError) * K_curve), velocity)
                  else:
                     self.pid_turn_right(velocity - (abs(turnError) * K_curve), velocity)
                  if ( 0.0 <= control <= 0.080):
                     print("Program should now stop")
                     self.stop()
               
                  break
         
                    


   def camera_values(self):
    while self.experiment_supervisor.step(self.timestep) != -1:
      # Get the list of recognizable objects
      objects = self.rgb_camera.getRecognitionObjects()
      
      # Iterate over each object
      for obj in objects:
      # Retrieve properties of the object
         # id = obj.getId()
         position = obj.getPosition()
         # orientation = obj.getOrientation()
         # size = obj.getSize()
         # position_on_image = obj.getPositionOnImage()
         # size_on_image = obj.getSizeOnImage()
         number_of_colors = obj.getNumberOfColors()
         colors = obj.getColors()
         # model = obj.getModel()
         #print("ID:", id)
         print("Position:", position[0], position[1])
         #print("Orientation:", orientation[0], orientation[1], orientation[2])
        # print("Size:", size[0], size[1])
         #print("Position on Image:", position_on_image[0], position_on_image[1])
         #print("Size on Image:", size_on_image[0], size_on_image[1])
         print("Number of Colors:", number_of_colors)
         print("Colors:", colors[0], colors[1], colors[2])
         #print("Model:", model)
   


   def rosbot_distance(self):
       return sum(self.get_encoder_readings())/4
   
   def pid_straight(self, velocity):
      self.set_rear_left_motor_velocity(velocity)
      self.set_rear_right_motor_velocity(velocity)
      self.set_front_left_motor_velocity(velocity)
      self.set_front_right_motor_velocity(velocity)
       

   def pid_turn_right(self, min_velocity, max_velocity):
     self.set_rear_left_motor_velocity(velocity=max_velocity)
     self.set_rear_right_motor_velocity(velocity=min_velocity)
     self.set_front_left_motor_velocity(velocity=max_velocity)
     self.set_front_right_motor_velocity(velocity=min_velocity)

   def pid_turn_left(self, min_velocity, max_velocity):
      self.set_rear_left_motor_velocity(velocity=min_velocity)
      self.set_rear_right_motor_velocity(velocity=max_velocity)
      self.set_front_left_motor_velocity(velocity=min_velocity)
      self.set_front_right_motor_velocity(velocity=max_velocity
                                          )
   def pid_wall_stop(self, leftOrRightSide, targetDistanceWall, targetDistanceSide, proportionalGainWall, proportionalGainSide):
       max_vel = 20
       min_vel = -20
       while self.experiment_supervisor.step(self.timestep) != -1:
         error = (-targetDistanceWall) - (-self.get_lidar_range_image()[400])
         errorSideL = (self.get_lidar_range_image()[200] - targetDistanceSide)
         errorSideR = (self.get_lidar_range_image()[600] - targetDistanceSide)
         control = error * proportionalGainWall
         print("left side distance reading:", self.get_lidar_range_image()[200])
         print("right side distance reading:", self.get_lidar_range_image()[600])
         print("front distance reading:", self.get_lidar_range_image()[400])
         if control > max_vel:
            velocity = max_vel
         elif min_vel <= control <= max_vel:
            velocity = control
         else:
            velocity = min_vel
         if leftOrRightSide == "left":
            if errorSideL > 0:
               self.pid_turn_left(velocity - (abs(errorSideL) * proportionalGainSide), velocity)
            else:
               self.pid_turn_right(velocity - (abs(errorSideL) * proportionalGainSide), velocity)
         else:
            if errorSideR > 0:
               self.pid_turn_right(velocity - (abs(errorSideR) * proportionalGainSide), velocity)
            else:
               self.pid_turn_left(velocity - (abs(errorSideR) * proportionalGainSide), velocity)     
         if ( 0.0 <= control <= 0.080):
               print("Program should now stop")
               self.stop()
               return 



   def pid_around_wall(self, proportional_gain, velocity, distance_from_wall, left_or_right):
      max_vel = 20
      min_vel = -20
      if left_or_right == "left":
         while self.experiment_supervisor.step(self.timestep) != -1:
            average_range_image = mean([
               self.get_lidar_range_image()[200],
               self.get_lidar_range_image()[225],
               self.get_lidar_range_image()[250],
               self.get_lidar_range_image()[275],
               ])
            error = (distance_from_wall) - average_range_image
            control = abs(error) * proportional_gain
            if self.get_lidar_range_image()[400] < 0.3:
               self.rotate_certain(90, 2, "right")
            elif -50 < error < -0.8:   
                  self.circular_rotation(math.pi/2, .5, velocity, "left")
            elif error > 0:
                  self.pid_turn_right(velocity - control, velocity)
            else:
                  self.pid_turn_left(velocity - control, velocity)
      
      else:
         while self.experiment_supervisor.step(self.timestep) != -1:
            average_range_image = mean([
               #self.get_lidar_range_image()[525],
               self.get_lidar_range_image()[550],
               self.get_lidar_range_image()[600],
               self.get_lidar_range_image()[575]
               ])
            error = (distance_from_wall) - average_range_image
            control = abs(error) * proportional_gain
            print(average_range_image)
            print("error:", error)
            print("control:", control)
            if self.get_lidar_range_image()[400] < 0.3:
               self.rotate_certain(90, 2, "left")
            elif float('-inf') <= error < -0.8:   
                  self.circular_rotation(math.pi/2, .5, velocity, "right")
            elif error > 0:
                  self.pid_turn_left(velocity - control, velocity)
            else:
                  self.pid_turn_right(velocity - control, velocity)
            
       
   def chill(self, second):
       start_time = self.experiment_supervisor.getTime()
       target_time = start_time + second
       while self.experiment_supervisor.step(self.timestep) != -1:
          if target_time <= self.experiment_supervisor.getTime():
             break



   def rotate_certain(self, target_rotation, vel, LoR):
    initial_rotation = self.get_compass_reading()
    accumulated_rotation = 0.0

    if LoR.lower() == "right":
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.set_rear_left_motor_velocity(velocity=vel)
            self.set_rear_right_motor_velocity(velocity=-vel)
            self.set_front_left_motor_velocity(velocity=vel)
            self.set_front_right_motor_velocity(velocity=-vel)

            current_rotation = self.get_compass_reading()
            accumulated_rotation = abs(current_rotation - initial_rotation)

            print("accumulated rotation:", accumulated_rotation)

            if accumulated_rotation >= target_rotation:
                self.stop()
                return
    else:
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.set_rear_left_motor_velocity(velocity=-vel)
            self.set_rear_right_motor_velocity(velocity=vel)
            self.set_front_left_motor_velocity(velocity=-vel)
            self.set_front_right_motor_velocity(velocity=vel)

            current_rotation = self.get_compass_reading()
            accumulated_rotation = abs(current_rotation - initial_rotation)

            print("accumulated rotation:", accumulated_rotation)

            if accumulated_rotation >= target_rotation:
                self.stop()
                return

   def rotate(self, degree, vel, margin_of_error=2):
    prev_encoder = sum(self.get_encoder_readings()) / len(self.get_encoder_readings()) * self.wheel_radius
    self.update_robot_pose(prev_encoder)
    target = (self.get_compass_reading() + degree) % 360  
    prev_encoder_reading = abs(self.get_front_left_motor_encoder_reading() * self.wheel_radius)
    # Determine the direction of rotation
    if degree > 0:
        # clockwise rotation
        self.set_rear_left_motor_velocity(velocity=-vel)
        self.set_rear_right_motor_velocity(velocity=vel)
        self.set_front_left_motor_velocity(velocity=-vel)
        self.set_front_right_motor_velocity(velocity=vel)
    else:
        # counter clockwise
        self.set_rear_left_motor_velocity(velocity=vel)
        self.set_rear_right_motor_velocity(velocity=-vel)
        self.set_front_left_motor_velocity(velocity=vel)
        self.set_front_right_motor_velocity(velocity=-vel)
    # Rotate until the target orientation is reached
    while self.experiment_supervisor.step(self.timestep) != -1:
        current = self.get_compass_reading()
        self.update_robot_pose(prev_encoder)
        prev_encoder = sum(self.get_encoder_readings()) / len(self.get_encoder_readings()) * self.wheel_radius
      #   print("rotate distance:", abs((self.wheel_radius * self.get_front_left_motor_encoder_reading()) - prev_encoder_reading))
        if (target - margin_of_error) <= current <= (target + margin_of_error):
            print("Reached target orientation: ", current)
            self.stop()
            return 
   # def rotate(self, degree, vel, LoR):
   #       if LoR == "Right" or LoR == "right":
   #          while self.experiment_supervisor.step(self.timestep) != -1:
   #             self.set_rear_left_motor_velocity(velocity=vel)
   #             self.set_rear_right_motor_velocity(velocity=-vel)
   #             self.set_front_left_motor_velocity(velocity=vel)
   #             self.set_front_right_motor_velocity(velocity=-vel)
   #             current_rotation = self.get_compass_reading()
   #             print("current rotation:", current_rotation)
   #             if current_rotation == degree:
   #                self.stop()
   #                return
   #       else:
   #          while self.experiment_supervisor.step(self.timestep) != -1:
   #             self.set_rear_left_motor_velocity(velocity=-vel)
   #             self.set_rear_right_motor_velocity(velocity=vel)
   #             self.set_front_left_motor_velocity(velocity=-vel)
   #             self.set_front_right_motor_velocity(velocity=vel)
   #             current_rotation = self.get_compass_reading()
   #             print("current rotation:", current_rotation)
   #             if current_rotation == degree:
   #                   self.stop()
   #                   return

   def move_forward(self, distance, vel):
      left_motor_extra = self.get_front_left_motor_encoder_reading() * self.wheel_radius  # wheel displacement that we do not need in calculations
      older_distance = sum(self.get_encoder_readings()) / len(self.get_encoder_readings()) * self.wheel_radius

      # time = distance/vel
      # old_time = self.experiment_supervisor.getTime()
      while self.experiment_supervisor.step(self.timestep) != -1:
        self.update_robot_pose(older_distance)
        older_distance = sum(self.get_encoder_readings()) / len(self.get_encoder_readings()) * self.wheel_radius
        distance_traveled = (self.wheel_radius * self.get_front_left_motor_encoder_reading()) - (left_motor_extra) 
        self.set_rear_left_motor_velocity(velocity=vel)
        self.set_rear_right_motor_velocity(velocity=vel)
        self.set_front_left_motor_velocity(velocity=vel)
        self.set_front_right_motor_velocity(velocity=vel)
      #   print("Velocity of left Wheel:", vel)
      #   print("Velocity of Right Wheel:", vel)
      #   print("Time it should take to travel:", time)
      #   #print("Time of travel in Virutal Time:", self.experiment_supervisor.getTime() - old_time)
      #   print("Distance for RosBot:", distance)
      #   print("Distance Traveled:", distance_traveled)
      #   if self.get_lidar_range_image()[400] < 0.45:
      #       self.rotate(180, 5)
      #       return
        if distance - self.margin_of_error_straight(vel) <= distance_traveled <= distance + self.margin_of_error_straight(vel):
          self.stop()
          return
    
   def margin_of_error_straight(self, velocity):
       if velocity >= 15:
          return 0.0985
       elif velocity >= 10:
          return 0.0300
       else:
          return 0.014
   def margin_of_error_rotate(self, velocity):
        if velocity >= 15:
          return 0.1200
        elif velocity >= 10:
          return 0.0385
        else:
          return 0.0100
       

   def circular_rotation(self, circle_angle, circle_radius, max_velocity, left_or_right):
          d_mid = 0.1325
          angular_velocity = (max_velocity/(circle_radius + d_mid))
          smaller_velocity = angular_velocity*(circle_radius - d_mid)
          left_motor_extra = self.get_front_left_motor_encoder_reading() * self.wheel_radius
          right_motor_extra = self.get_front_right_motor_encoder_reading() * self.wheel_radius
          #motor_encoder_extra = (left_motor_extra + right_motor_extra)/2
          distance = (circle_angle*circle_radius)
          #distance_average_current = sum(self.get_encoder_readings())/4
          time = distance/((max_velocity+smaller_velocity)/2)
          old_time = self.experiment_supervisor.getTime()
          if left_or_right == "right":
            left_wheel_distance = circle_angle*(circle_radius + 0.1325)
            while self.experiment_supervisor.step(self.timestep) != -1:
               #sum(self.get_encoder_readings())/4
               distance_traveled = (self.wheel_radius * self.get_front_left_motor_encoder_reading()) - (left_motor_extra)
               #distance_rosbot = (sum(self.get_encoder_readings())/4) - distance_average_current
               self.set_rear_left_motor_velocity(velocity=max_velocity)
               self.set_rear_right_motor_velocity(velocity=smaller_velocity)
               self.set_front_left_motor_velocity(velocity=max_velocity)
               self.set_front_right_motor_velocity(velocity=smaller_velocity)
               print("Velocity of left Wheel:", max_velocity)
               print("Velocity of Right Wheel:", smaller_velocity)
               print("Time it should take to travel:", time)
               #print("Time of travel in Virutal Time:", self.experiment_supervisor.getTime() - old_time)
               print("Distance for RosBot:", distance)
               print("Distance for RosBot's Left Wheel:", left_wheel_distance)
               print("Distance Traveled: on RosBot's Left Wheel", distance_traveled)
               if left_wheel_distance - self.margin_of_error_rotate(max_velocity) <= distance_traveled <= left_wheel_distance + self.margin_of_error_rotate(max_velocity):
                  self.stop()
                  break
          else:
            right_wheel_distance = circle_angle*(circle_radius + 0.1325)
            while self.experiment_supervisor.step(self.timestep) != -1:
               distance_traveled = (self.wheel_radius * self.get_front_right_motor_encoder_reading()) - (right_motor_extra)
               #distance_rosbot = (sum(self.get_encoder_readings())/4) - distance_average_current
               self.set_rear_left_motor_velocity(velocity=smaller_velocity)
               self.set_rear_right_motor_velocity(velocity=max_velocity)
               self.set_front_left_motor_velocity(velocity=smaller_velocity)
               self.set_front_right_motor_velocity(velocity=max_velocity)
               print("Velocity of left Wheel:", max_velocity)
               print("Velocity of Right Wheel:", smaller_velocity)
               print("Time it should take to travel:", time)
               #print("Time of travel in Virutal Time:", self.experiment_supervisor.getTime() - old_time)
               print("Distance for RosBot:", distance)
               #print("Distance for RosBot's Right Wheel:", right_wheel_distance)
               print("Distance Traveled: on RosBot's Right Wheel", distance_traveled)

               if distance_traveled > right_wheel_distance:
                  self.stop()
                  break
                    

          
      # def pid_straight(self, proportional_gain, distance_wanted):
      #   min_vel = -20
      #   max_vel = 20
      #   while self.experiment_supervisor.step(self.timestep) != -1:
      #      error = (-distance_wanted) - (-self.get_lidar_range_image()[400])
      #      control = error * proportional_gain
      #      print(control)
      #      if control > max_vel:
      #        self.set_rear_left_motor_velocity(max_vel)
      #        self.set_rear_right_motor_velocity(max_vel)
      #        self.set_front_left_motor_velocity(max_vel)
      #        self.set_front_right_motor_velocity(max_vel)
      #      elif min_vel <= control <= max_vel:
      #        self.set_rear_left_motor_velocity(control)
      #        self.set_rear_right_motor_velocity(control)
      #        self.set_front_left_motor_velocity(control)
      #        self.set_front_right_motor_velocity(control)
      #      else:
      #        self.set_rear_left_motor_velocity(min_vel)
      #        self.set_rear_right_motor_velocity(min_vel)
      #        self.set_front_left_motor_velocity(min_vel)
      #        self.set_front_right_motor_velocity(min_vel)
      #      if ( 0.0 <= control <= 0.101):
      #         print("Program should now stop")
      #         self.stop()
      #         return