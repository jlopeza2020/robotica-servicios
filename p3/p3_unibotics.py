from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np


def align_car(difference, dis, aligned_start_time):
  
  aligned_duration_threshold = 5.0  # Ajusta este valor según tus necesidades
  reached = False
  #aligned_start_time = None
  
  if abs(difference) < 15 and (0.20 < dis < 0.45):
        
    if aligned_start_time is None:
      # Initial timestamp 
      aligned_start_time = time.time()
      print("Alineado.")
      HAL.setW(0.0)
      HAL.setV(0.0)
      
    else:
          # check if it has been align in specific time
      elapsed_time = time.time() - aligned_start_time
      if elapsed_time >= aligned_duration_threshold:
        print(f"Permaneció alineado durante {aligned_duration_threshold} segundos. Pasar al siguiente estado.")
        #align = False
        #find = True
        reached = True
            
      else:
        print(f"Permaneciendo alineado ({elapsed_time:.2f} segundos).")

  else:
       # No está alineado, reiniciar la marca de tiempo
    aligned_start_time = None

    if difference < 0:
      print("Girado hacia la izquierda.")
      HAL.setW(-1.0)
      HAL.setV(0.5)

    else:
      print("Girado hacia la derecha.")
      HAL.setW(1.0)
      HAL.setV(0.5)
  
  return reached, aligned_start_time
  

def move_ahead(init_y_pose, distance):
  
  reached = False
  
  #if(first_iter_pos): 
  #  init_y_pose = HAL.getPose3d().y
  #  first_iter_pos = False
        
  actual_y_pose = HAL.getPose3d().y
      
  diff_y_pose = actual_y_pose - init_y_pose
  # go ahead 6 meters 
  if(abs(diff_y_pose) <= 6.0):

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

    print("Hueco encontrado")
    reached = True
    #find = False
    #park_move_0 = True
    
  return reached

        
def get_distances_to_car(laser):

  values = []
  for dist , angle in laser:
        
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
    len_laser = len(laser_data.values)

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

#ocuppied = False 

align = True
find = False
park_move_0 = False
park_move_1 = False

# cuadrado para encontrar hueco 
y = 7
x = 5 
betha = get_betha(x,y)


first_iter_pos = True
first_iter_turn = True

aligned_start_time = None
aligned_duration_threshold = 5.0  # Ajusta este valor según tus necesidades


while True:
    
    #front_laser = HAL.getFrontLaserData()
    #parse_front_laser = parse_laser_data(front_laser)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    distances_cars = get_distances_to_car(parse_right_laser)
    
    #back_laser = HAL.getBackLaserData()
    #parse_back_laser = parse_laser_data(back_laser)
    
    # STATE 1: ALIGN 
    if (align): 
      

      dis = calculate_std(distances_cars)
      difference = compare_sides(distances_cars)
      
      print(difference, dis)  
      
      find, aligned_start_time = align_car(difference, dis, aligned_start_time)
      """
      if abs(difference) < 15 and (0.20 < dis < 0.45):
        if aligned_start_time is None:

          # Initial timestamp 
          aligned_start_time = time.time()
          print("Alineado.")
          HAL.setW(0.0)
          HAL.setV(0.0)
        else:
          # check if it has been align in specific time
          elapsed_time = time.time() - aligned_start_time
          if elapsed_time >= aligned_duration_threshold:
            print(f"Permaneció alineado durante {aligned_duration_threshold} segundos. Pasar al siguiente estado.")
            align = False
            find = True
            
          else:
            print(f"Permaneciendo alineado ({elapsed_time:.2f} segundos).")

      else:
       # No está alineado, reiniciar la marca de tiempo
        aligned_start_time = None

        if difference < 0:
          print("Girado hacia la izquierda.")
          HAL.setW(-1.0)
          HAL.setV(0.5)

        else:
          print("Girado hacia la derecha.")
          HAL.setW(1.0)
          HAL.setV(0.5)
        
      """
     # STATE 2: FIND SPACE
    if (find):
      
      align = False
      park_move_0 = find_space(parse_right_laser, betha)
      

    # STATE 3: MOVE 6 METERS AHEAD
    if(park_move_0):
      
      find = False
      
      if(first_iter_pos): 
        init_y_pose = HAL.getPose3d().y
        first_iter_pos = False
        
      park_move_1 = move_ahead(init_y_pose, 6)
      
    # STATE 4: TURN 45 METERS 
    if(park_move_1):
      
      park_move_0 = False
      
      print(HAL.getPose3d().yaw)
      HAL.setV(0.0)
      HAL.setW(0.0)
      
      
    