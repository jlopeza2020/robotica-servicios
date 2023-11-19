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

#alpha = 0
y = 1.2
x = 2.64 
ocuppied = False 
init_range = 1.0472 # 60º
end_range = 2.0944 # 120º

while True:
    front_laser = HAL.getFrontLaserData()
    parse_front_laser = parse_laser_data(front_laser)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
   # print(parse_right_laser)
    
    back_laser = HAL.getBackLaserData()
    parse_back_laser = parse_laser_data(back_laser)
    
    #init_range, end_range = calculate_range_laser(x, y)
    
    
    #ocuppied = 100 not in [dist for dist, angle in parse_right_laser if init_range <= angle <= end_range]

    #print("Occupied:", ocuppied)
    
    
    
    #time.sleep(1)  # Add a delay to control the loop frequency

    for measurement in parse_right_laser: 
        if init_range <= measurement[1] <= end_range and measurement[0] <= 5:
            ocuppied = True
            break

    if ocuppied:
        print("No hay hueco")
        ocuppied = False
        HAL.setV(0.3)
    else:
        all_measurements_are_100 = all(measurement[0] > 5 for measurement in parse_right_laser)
        if all_measurements_are_100:
            print("Sí hay hueco")
            HAL.setV(0.0)
        #else:
        #    print("Algunas medidas dentro del rango no son 100")
            
    
    #.setV(0.3)
    

    