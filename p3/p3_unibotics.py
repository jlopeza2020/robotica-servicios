from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np


def calcular_pendiente(right_laser):
  
  
  #values_left = []
  #values_right = []
  values = []
  
  for dist , angle in right_laser:
        
    x = math.cos(angle) * dist
    y = math.sin(angle) * dist
    
    # 0-90
    #if(math.radians(70) < angle < math.pi/2):
    #  if(dist != 100):
    #    values_right.append((x,y))
      
    #if(math.pi/2 < angle < math.radians(110)):
      # 90-180
    #  if(dist != 100):
    #    values_left.append((x,y))
    #if(dist != 100):
    
    
    if(math.radians(70) < angle < math.radians(110)):

      values.append((x,y))

    # left
    #if(math.radians(init_range) < angle < math.pi/2): 
    #values.append(math.sin(angle) * dist)

    # right 
    #if(math.pi/2 < angle < math.radians(end_range)):
    #values.append((x,y))

    # Extraer las coordenadas x e y de los puntos
 
  x_values = np.array([p[0] for p in values])
  y_values = np.array([p[1] for p in values])
  
  pendiente = np.diff(y_values) / np.diff(x_values)

  """
  x_right_values = np.array([p[0] for p in values_right])
  y_right_values = np.array([p[1] for p in values_right])
  
  pendiente_right = np.diff(y_right_values) / np.diff(x_right_values)
  
  x_left_values = np.array([p[0] for p in values_left])
  y_left_values = np.array([p[1] for p in values_left])

    # Calcular la pendiente usando la fórmula (cambio en y) / (cambio en x)
  pendiente_left = np.diff(y_left_values) / np.diff(x_left_values)

  # Devolver la pendiente promedio si hay más de un punto
  return np.mean(pendiente_left), np.mean(pendiente_right)

  """
  return np.mean(pendiente)

# if std is low: small differences between measures
# if std is high: big differences between measures
def get_std_ranges(right_laser,init_range, end_range):
  
  left_values = []
  right_values = []
      
  for dist , angle in right_laser:
        
    # left
    if(math.radians(init_range) < angle < math.pi/2): 
      right_values.append(math.sin(angle) * dist)

    # right 
    if(math.pi/2 < angle < math.radians(end_range)):
      left_values.append(math.sin(angle) * dist)

  std_left = np.std(left_values)
  std_right = np.std(right_values)
  
  return std_left, std_right
  

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
      #pte_left, pte_right = calcular_pendiente(parse_right_laser)

      print("Pendiente: ", pte)
      
      if (-0.40 < pte < -0.3):
        print("alineado")
        HAL.setW(0.0)
        align = False
        find = True
        
      if(pte < -0.4):
        HAL.setW(0.25)
        
      if(pte > -0.3):
        HAL.setW(-0.25)
      
 
      #std_left, std_right = get_std_ranges(parse_right_laser, 45, 135)

      
      # check if it is align
      #if(is_turned_back == False and is_turned_front == False):
        
      """
        if(std_left <= 0.3 and std_right  <= 0.3):
          print("Estoy alineado")
          HAL.setW(0.0)
          find = True
          # find and align at the same time 
          # go ahead and find space
        if(std_left <= 0.3 and std_right  > 0.3):
          print("right bigger")
          HAL.setW(0.25)
          
        if(std_left > 0.3 and std_right  <= 0.3):
          print("left bigger")
          HAL.setW(-0.25)



        if (std_left > 0.3 and std_right > 0.3):
         
          if(std_left > std_right):
            print("left bigger")
            HAL.setW(-0.25)
            
          if(std_left < std_right):
            print("right bigger")
            HAL.setW(0.25)
          
          if(std_left == std_right):
            print("alineado")
            HAL.setW(0.0)
          

        """  
      """
      # need to robust movement and set threshold 
      # start moving until threshold 
      if(is_turned_back and is_turned_front == False):
        print("is turn back and amount " + str(amount_bl_af))
        HAL.setW(-0.5)
        
      if(is_turned_front and is_turned_back == False):
        print("is turn front and amount "+ str(amount_fl_af))
        HAL.setW(0.5)
        
      """  
        
      #print(std_left, std_right, is_turned_front, amount_fl_af, is_turned_back, amount_bl_af)

        
     # STATE 2: FIND SPACE
    if (find):
      #align = False
      
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
        align = False
        HAL.setV(0.0)
        HAL.setW(0.0)
      
      
      
      
      
    