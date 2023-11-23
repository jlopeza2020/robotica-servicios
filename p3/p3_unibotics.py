from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np

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

y = 7
x = 5 
betha = get_betha(x,y)

while True:
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    
    # STATE 1: ALIGN 
    
    
    # STATE 2: FIND SPACE
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
      
      
      
      
      
    