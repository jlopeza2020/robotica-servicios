from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np

def move_forward_turn(front_distances):

  reached = False
  close_front = False
  
  for i in front_distances:
    if(i < 0.3): 
      close_front = True
      break
        
  if(not close_front):

    HAL.setW(-5.0)
    HAL.setV(1.0)
    
  else: 
    reached = True
    
  return reached
  
  
def move_back_turn(back_distances):

  reached = False
  close_back = False
  
  for i in back_distances:
    if(i < 0.3): 
      close_back = True
      break
        
  if(not close_back):

    HAL.setW(-5.0)
    HAL.setV(-2.0)
    
  else: 
    reached = True
    
  return reached
  

def move_back(laser_distances, init_x_pose, distance):
  reached = False
  close = False

  actual_x_pose = HAL.getPose3d().x
  diff_x_pose = actual_x_pose - init_x_pose

  if(np.mean(laser_distances) >= 1.0 and abs(diff_x_pose) <= distance):

    HAL.setV(-1.0)
    HAL.setW(0.0)
  else:
    reached = True
  
  return reached


def turn(init_yaw, angle):

  reached = False

  actual_yaw = HAL.getPose3d().yaw
  diff_yaw = actual_yaw - init_yaw
  
  if(abs(diff_yaw) <= angle):

    HAL.setW(1.0)
    HAL.setV(0.75)
    
  else: 
    reached = True
    
  return reached
    
    
def move_ahead(init_y_pose, distance):
  
  reached = False
  
  actual_y_pose = HAL.getPose3d().y
      
  diff_y_pose = actual_y_pose - init_y_pose
 
  if(abs(diff_y_pose) <= distance):

    HAL.setV(0.75)
    HAL.setW(0.0)
    
  else: 
    reached = True
    
  return reached
    
    
def find_space(laser, betha):
  
  reached = False
  ocuppied = False 
  
  for d_actual, angle in parse_right_laser:
    # values from the upper part of the rectangle
    if(betha < angle < math.pi - betha):
      d_objective = y / math.sin(angle)
    else: 
      d_objective = (x/2) / math.cos(angle)
        
    # means that there is an obstacle in the rectangle
    if d_actual < d_objective:
      ocuppied = True
      break
        
  if (ocuppied):
    HAL.setV(0.75)
    HAL.setW(0.0)
    ocuppied = False
      
  else:

    print("Space found")
    reached = True

  return reached


def align_car(difference, dis, aligned_start_time):
  
  aligned_duration_threshold = 5.0
  reached = False

  if abs(difference) < 10 and (0.20 < dis < 0.45):
        
    if aligned_start_time is None:
      # Initial timestamp 
      aligned_start_time = time.time()
      print("Aligned.")
      
    else:
      # check if it has been align in specific time
      elapsed_time = time.time() - aligned_start_time
      if elapsed_time >= aligned_duration_threshold:
        print(f"Stayed aligned along {aligned_duration_threshold} secs. Go to next step.")

        reached = True
            
      else:
        print(f"Stay align ({elapsed_time:.2f} secs).")
    
    HAL.setW(0.0)
    HAL.setV(0.0)

  else:
    # if not align, restart timestamp
    aligned_start_time = None

    if difference < 0:
      print("Turned to the left.")
      HAL.setW(-1.0)
      HAL.setV(0.5)

    else:
      print("Turned to the right.")
      HAL.setW(1.0)
      HAL.setV(0.5)
  
  return reached, aligned_start_time
  
      
def get_distances_to_car(laser, init_angle, end_angle):
  
  values = []
  for dist , angle in laser:
        
    if(init_angle < angle < end_angle):
      y = math.sin(angle) * dist
      values.append(y)

  return values


def compare_sides(values):

  half = len(values) // 2
  side_left = values[:half]
  side_right = values[half:]

  # calculate sum of the measures of both sides
  sum_side_left = sum(side_left)
  sum_side_right = sum(side_right)
  
  diff = sum_side_left - sum_side_right
  
  return diff


def calculate_std(values):
  return np.std(values)


def get_betha(a, b):
  
  h = math.sqrt(pow(a/2, 2) + pow(b,2))
  betha = math.acos((a/2)/h)
  
  return betha
  

def parse_laser_data(laser_data):
  
  laser = []

  # store values if length is different from 0
  if(laser_data.values):
    for i in range(180):
        # if value is inside of the minimum and maximun 
      if(laser_data.minRange < laser_data.values[i] and laser_data.values[i] < laser_data.maxRange):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
          
  return laser
   

print("xxxxxxxxxxxxxxxxxxxxxxxxxxxx")

# states
align = True
find = False
park_move_0 = False
park_move_1 = False
park_move_2 = False
park_move_3_4 = False

# square defined to find space
y = 7
x = 5 
betha = get_betha(x,y)

first_iter_pos_y = True
first_iter_turn = True
first_iter_pos_x = True

aligned_start_time = None
distance_ahead = 5.0
angle_turn = 0.6
distance_back = 5.0 

estado = 3
goal_yaw = 0.0

while True:
    
    front_laser = HAL.getFrontLaserData()
    parse_front_laser = parse_laser_data(front_laser)
    
    distances_front_cars = get_distances_to_car(parse_front_laser, 0, math.pi)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    distances_right_cars = get_distances_to_car(parse_right_laser, 0, math.pi)
    
    back_laser = HAL.getBackLaserData()
    parse_back_laser = parse_laser_data(back_laser)
    
    distances_back_cars = get_distances_to_car(parse_back_laser, 0, math.pi)
    
    # STATE 1: ALIGN 
    if (align): 
      
      dis = calculate_std(distances_right_cars)
      difference = compare_sides(distances_right_cars)
      
      find, aligned_start_time = align_car(difference, dis, aligned_start_time)
     
    # STATE 2: FIND SPACE
    if (find):
      
      align = False
      park_move_0 = find_space(parse_right_laser, betha)
      
    # STATE 3: MOVE 6 METERS AHEAD
    if(park_move_0):
      
      find = False
      
      if(first_iter_pos_y): 
        init_y_pose = HAL.getPose3d().y
        first_iter_pos_y = False
        
      park_move_1 = move_ahead(init_y_pose, distance_ahead)
      
    # STATE 4: TURN 45 METERS 
    if(park_move_1):
      park_move_0 = False
      
      if(first_iter_turn): 
        init_y_yaw = HAL.getPose3d().yaw
        first_iter_turn = False
      
      park_move_2 = turn(init_y_yaw, angle_turn)

    # STATE 5: GO BACK UNTIL DETECT CAR OR ODOMETRY 
    if(park_move_2):
      
      park_move_1 = False
      
      if(first_iter_pos_x): 
        init_x_pose = HAL.getPose3d().x
        first_iter_pos_x = False
        
      park_move_3_4 = move_back(distances_back_cars, init_x_pose, distance_back)
      
    # STATE 6 AND 7: MOVES TO PARK
    if park_move_3_4: 
      
      park_move_2 = False  
      actual_yaw = HAL.getPose3d().yaw
      diff_yaw = actual_yaw - goal_yaw
    
      if diff_yaw >= 0.05:
        
        if(estado == 3):

          if(move_back_turn(distances_back_cars)):
            HAL.setV(0.0)
            HAL.setW(0.0)
            estado = 4
        else:    
          
          if(move_forward_turn(distances_front_cars)):

            HAL.setV(0.0)
            HAL.setW(0.0)
            estado = 3
            
      else:
        
        park_move_3_4 = False
        HAL.setV(0.0)
        HAL.setW(0.0)


  