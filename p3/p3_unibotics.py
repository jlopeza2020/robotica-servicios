from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np


def get_right_straight_means(parse_right_laser,init_degree, end_degree):
  
  value_left = 0 
  count_left = 0
  value_right = 0 
  count_right = 0 
      
  for dist , angle in parse_right_laser:
        
    # left
    if(math.radians(init_degree) < angle < math.pi/2): 

      value_left += dist
      count_left += 1
        
    # right 
    if(math.pi/2 < angle < math.radians(end_degree)):
      value_right += dist
      count_right += 1
      

      
  mean_left = value_left/count_left
  mean_right = value_right/count_right
  
  return mean_left, mean_right
  

def get_back_straight(back_laser):
  
  is_turned = False
  
  for dist , angle in back_laser:
    
    if (angle <= math.pi/2):
      if(dist != 100): 
        is_turned = True
        break
          #else: 
            
      #print("value front"+ str(no_cien))
  return is_turned


def get_front_straight(front_laser):
  
  is_turned = False
  
  for dist , angle in front_laser:
    
    if (angle >= math.pi/2):
      #print(angle, dist)
      
      if(dist != 100): 
        is_turned = True
        break
          #else: 
            
      #print("value front"+ str(no_cien))
  return is_turned
  
      
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
        else: 
          dist = 100 # means that 100 = inf
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser
   
print("xxxxxxxxxxxxxxxxxxxxxxxxxxxx")

ocuppied = False 

align = True
find = False 


#init_degree_detection  = 60
#end_degree_detection = 120

#convert degrees into radian
#init_radian_detection = math.radians(init_degree_detection)
#end_radian_detection = math.radians(end_degree_detection)
#m_left, m_right = get_right_straight_means(parse_right_laser,80, 100)

y = 7
x = 5 
betha = get_betha(x,y)


while True:
    
    front_laser = HAL.getFrontLaserData()
    parse_front_laser = parse_laser_data(front_laser)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    back_laser = HAL.getBackLaserData()
    parse_back_laser = parse_laser_data(back_laser)
    
    # STATE 1: ALIGN 
    if (align): 
      
      # APPROACH 1: 
      # FRONT LASER IS NOT WELL PLACED !!!!
      
      #is_turned_front = get_front_straight(parse_front_laser)
      #is_turned_back = get_back_straight(parse_back_laser)
      #m_left, m_right = get_right_straight_means(parse_right_laser,80, 100)
      
      # APPROACH 2:
      
      for dist, angle in parse_right_laser:
        
        x = math.cos(angle)*dist
        
        y = math.sin(angle)*dist
      
        
        
     # STATE 2: FIND SPACE
    if (find):
      align = False
      
      for d_actual, angle in parse_right_laser:
      # values from the upper part of the rectangle
        if(betha < angle < math.pi - betha):
          d_objective = y / math.sin(angle)
        else: 
          d_objective = (x/2) / math.cos(angle)
        
      # meanss that there is an obstacle in the rectangle
        if d_actual < d_objective:
          ocuppied = True
          break
        
      if (ocuppied):
        HAL.setV(0.5)
        ocuppied = False
      
      else: 
        print("HAY HUECO")
        HAL.setV(0.0)
      
      
      
      
      
    