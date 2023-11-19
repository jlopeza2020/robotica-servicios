from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np

# FASE DE APROXIMACION Y ALINEACIÓN
# BÚSSQUEDA 
# APARCAMIENTO

def calculate_range_laser(x, y):
  
  # for 0 - 90ª
  
  init_angle = math.atan2(y, x)
  
  # for 90 - 180 
  end_angle = math.pi - math.atan2(y, x)
  
  
  return init_angle, end_angle
  
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

alpha = 0
y = 1.2
x =  2.64 
check_free = 0
while True:
    front_laser = HAL.getFrontLaserData()
    parse_front_laser = parse_laser_data(front_laser)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    back_laser = HAL.getBackLaserData()
    parse_back_laser = parse_laser_data(back_laser)
    
    init_range, end_range = calculate_range_laser(x,y)
    
    #print(init_range, end_range)
    # means 0-90
    for i in range (len(parse_right_laser)): 
      
      if(init_range <= parse_right_laser[i][1] and parse_right_laser[i][1] >= end_range and parse_right_laser[i][0] != 100):
        ocuppied = True
        break
      
    if(ocuppied): 
      print("No hay hueco")
      ocuppied = False
      

      
    
    
    

    