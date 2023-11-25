from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np


def calcular_pendiente(right_laser):
  
  values = []
  
  for dist , angle in right_laser:
        
    x = math.cos(angle) * dist
    y = math.sin(angle) * dist
   
    if(math.radians(80) < angle < math.radians(100)):
      values.append((x,y))

  x_values = np.array([p[0] for p in values])
  y_values = np.array([p[1] for p in values])
  
  pendiente = np.diff(y_values) / np.diff(x_values)

 
  return np.mean(pendiente)


def get_back_straight(back_laser):
  
  is_turned = False
  times = 0
  
  for dist , angle in back_laser:
    
    if (angle <= math.pi/2):
      if(dist != 100): 
        times += 1
        
      
  
  for dist , angle in back_laser:
    
    if (angle <= math.pi/2):
      if(dist != 100): 
        is_turned = True
        break

  return is_turned , times


def get_front_straight(front_laser):
  
  is_turned = False
  times = 0
  for dist , angle in front_laser:
    
    if (angle >= math.pi/2):
      
      if(dist != 100): 
        times += 1
  
  for dist , angle in front_laser:
    
    if (angle >= math.pi/2):
      
      if(dist != 100): 
        is_turned = True
        break

  return is_turned , times
  
      
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

y = 7
x = 5 
betha = get_betha(x,y)


first_iter = True

# Inicializa la variable que almacenará el valor deseado
init_pte = None

while True:
    
    front_laser = HAL.getFrontLaserData()
    parse_front_laser = parse_laser_data(front_laser)
    
    right_laser = HAL.getRightLaserData()
    parse_right_laser = parse_laser_data(right_laser)
    
    back_laser = HAL.getBackLaserData()
    parse_back_laser = parse_laser_data(back_laser)
    
    # STATE 1: ALIGN 
    if (align): 
      

      is_turned_front, amount_fl_af = get_front_straight(parse_front_laser)
      is_turned_back, amount_bl_af = get_back_straight(parse_back_laser)
      
      pte = calcular_pendiente(parse_right_laser)
      

      if first_iter:

        init_pte = calcular_pendiente(parse_right_laser) 
        first_iter = False  
      
      if (init_pte -0.1 < pte < init_pte + 0.1):
        print("alineado")
        HAL.setW(0.0)
        #align = False
        #find = True
        
      #if(pte < init_pte -0.1):
        #HAL.setW(0.25)
        
      #if(pte > init_pte + 0.1):
        #HAL.setW(-0.25)
        
        
      diff_pte = pte - init_pte
      print(init_pte, pte, diff_pte, HAL.getPose3d().yaw)
      
      # need to robust movement and set threshold 
      # start moving until threshold 
      if(is_turned_back and is_turned_front == False):
        print("is turn back and amount " + str(amount_bl_af))
        HAL.setW(-0.5)
        
      if(is_turned_front and is_turned_back == False):
        print("is turn front and amount "+ str(amount_fl_af))
        HAL.setW(0.5)
        
        
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
      
      
      
      
      
    