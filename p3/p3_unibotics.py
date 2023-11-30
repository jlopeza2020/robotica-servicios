from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np


def get_distances_to_car(laser):

  values = []
  #counter = 0
  for dist , angle in laser:
        
    y = math.sin(angle) * dist
    values.append(y)
    #counter += 1
    
    
  #if (counter >= 45):
  return values
 
        

def compare_sides(values):

  #values = []
  
  #for dist , angle in laser:
        
    #y = math.sin(angle) * dist
    #values.append(y)
    
  half = len(values) // 2
  side_left = values[:half]
  side_right = values[half:]

    # Calcular la suma de las medidas para cada lado
  sum_side_left = sum(side_left)
  sum_side_right = sum(side_right)
  
  diff = sum_side_left - sum_side_right
  
  
  return diff
  #print(suma_lado_izquierdo, suma_lado_derecho)
  """
  if(abs(diff) < 15):
    return "Alineado."
  else: 
    
    if diff < 0:
     return "girado hacia la izquierda."
     
    else:
      return "girado hacia la derecha"

  """

def calcular_dispersion(values):
  
  #values = []
  
  #for dist , angle in right_laser:
        
  #  y = math.sin(angle) * dist
   
    #values.append(y)

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

ocuppied = False 

align = True
find = False

# cuadrado para encontrar hueco 
y = 7
x = 5 
betha = get_betha(x,y)


first_iter = True
#init_pte = None

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
      

      dis = calcular_dispersion(distances_cars)
      

      difference = compare_sides(distances_cars)
      print(difference, dis)  
      
      
      if(abs(difference) < 15 and (0.20 < dis < 0.4)):
        print("Alineado.")
      else: 
    
        if difference < 0:
          print("girado hacia la izquierda.")
          
          HAL.setW(-1.0)
          HAL.setV(0.5)
     
        else:
          print("girado hacia la derecha")
      
      
      # if yaw es positiva: está girada hacia la izq 
      # si yaw es negativa está girada a la derecha 
      
      
      
      
        
     # STATE 2: FIND SPACE
    if (find):
      
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
        HAL.setV(0.75)
        ocuppied = False
      
      else: 
        print("HAY HUECO")
        HAL.setV(0.0)
        HAL.setW(0.0)
      
      
      
      
      
    