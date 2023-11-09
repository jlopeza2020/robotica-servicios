from GUI import GUI
from HAL import HAL
import numpy as np


def get_difference(objective, actual): 
  
    if (objective < actual): 
      dif = actual - objective
    else:
      dif = objective - actual
      
    return dif
  
actual_height = 0.0
goal_height = 2.0

# Previously converted corrdinates from GPS to UTM
east_boat_utm = 430492.2
north_boat_utm = 4459161.1

east_survivors_utm = 430532.5
north_survivors_utm = 4459131.8

east_survivors2boat = 430532.5 - 430492.2
goal_x = east_survivors2boat
#print(east_survivors2boat)
north_survivors2boat = 4459131.8 - 4459161.1
goal_y = north_survivors2boat

goal_yaw = np.arctan2(goal_y , goal_x)
#print(north_survivors2boat)

# Must be 6
num_survivors_found = 0
total_survivors = 6

# x
#print(HAL.get_position()[0])
# y
#print(HAL.get_position()[1])
# z
#print(HAL.get_position()[2])

phase_1 = True 
phase_2 = False
while True:
    # state 1 
    actual_x = HAL.get_position()[0]
    actual_y = HAL.get_position()[1]
    actual_height = HAL.get_position()[2]
    actual_yaw = HAL.get_yaw()
    
    
    diff_x = get_difference(goal_x, actual_x)
    diff_y = get_difference(goal_y,  actual_y)
    diff_height = get_difference(goal_height, actual_height)
    diff_yaw = get_difference(goal_yaw, actual_yaw)

    if(phase_1): 
      if(diff_height > 0.1):
        HAL.takeoff(goal_height)
      else: 
        phase_1 = False
        phase_2 = True
    
    # = goal_height - actual_height
    
    #if (dif_height > 0.25):
      #HAL.takeoff(goal_height)
      #print("estoy despegando")
    #else: 
    #  print("ya he terminado de despegar")
      
    # state 2 
    
    if(phase_2): 
      if(diff_x > 0.1 and diff_y > 0.1 and  diff_height < 0.1 and diff_yaw > 0.01):
      #phase_1 = False
      #if()
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_2 = False
      
      
    # Enter iterative code!
    print("++++++++++++++++++++++++++++++++++++++++++")
    print("actual")
    print(actual_x, actual_y, actual_height, actual_yaw)
    print("goal")
    print(goal_x, goal_y, goal_height, goal_yaw)
    print("dif")
    print(diff_x, diff_y, diff_height, diff_yaw)
    
    # Show ventral and frontal image
    frontal_image = HAL.get_frontal_image()
    GUI.showImage(frontal_image)
    ventral_image = HAL.get_ventral_image()
    GUI.showLeftImage(ventral_image)