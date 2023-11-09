from GUI import GUI
from HAL import HAL
import numpy as np
import math


def generate_spiral_square(center, num_turns):
    waypoints = []
    
    current_point = np.array(center)
    waypoints.append(tuple(current_point))
    incremento = 1
    
    # Generar la espiral cuadrada
    for _ in range(num_turns):
      
        # primera (x, y + 1)
        if(current_point[1] > 0):
          current_point[1] += incremento
        else:
          current_point[1] -= incremento
            
        waypoints.append(tuple(current_point))
        incremento += 1
            
        # segunda(x-2, y) 
        if(current_point[0] > 0):
          current_point[0] -= incremento
        else:
          current_point[0] += incremento
          
        waypoints.append(tuple(current_point))
        incremento += 1
            
        # tercera (x, y - 3)
        if(current_point[1] > 0):
          current_point[1] -= incremento
        else: 
          current_point[1] += incremento
          
        waypoints.append(tuple(current_point))
        incremento += 1
              
        if(current_point[0] > 0):
          current_point[0] += incremento
        else:
          current_point[0] -= incremento
          
        waypoints.append(tuple(current_point))
        incremento += 1
  
    return waypoints
    
def get_difference(objective, actual): 
    if (objective < actual): 
      dif = actual - objective
    else:
      dif = objective - actual
      
    return dif
  
actual_height = 0.0
goal_height = 2.5

# Previously converted corrdinates from GPS to UTM
east_boat_utm = 430492.2
north_boat_utm = 4459161.1

east_survivors_utm = 430532.5
north_survivors_utm = 4459131.8

# calculate distance from goal to actual
east_survivors2boat = east_survivors_utm - east_boat_utm
goal_x = east_survivors2boat

north_survivors2boat = north_survivors_utm - north_boat_utm
goal_y = north_survivors2boat

# calculate goal yaw 
goal_yaw = np.arctan2(goal_y , goal_x)

# Must be 6
num_survivors_found = 0
total_survivors = 6

phase_take_off = True  
phase_go_to_survivors = False
phase_finding = False


num_pos_waypoints = 0
# Definir el centro, longitud del lado inicial, número de vueltas y tamaño del paso
center = [goal_x, goal_y]
num_turns = 10
#step_size = 2

# Generar los waypoints de la espiral cuadrada
waypoints_list = generate_spiral_square(center, num_turns)

Kp = 1.5
#for point in waypoints_list:
#    print(point)
print("++++++++++++++++++++++++++++++++++++++++++")
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

    if(phase_take_off): 
      if(diff_height > 0.1):
        HAL.takeoff(goal_height)
      else: 
        phase_take_off = False
        phase_go_to_survivors = True
    

    if(phase_go_to_survivors): 
      if(diff_x > 0.1 or diff_y > 0.1 or  diff_height > 0.1 or diff_yaw > 0.01):
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_go_to_survivors = False
        phase_finding = True
        
    if(phase_finding):
      if(num_pos_waypoints < len(waypoints_list)):
        
        goal_x_waypoint = waypoints_list[num_pos_waypoints][0]
        goal_y_waypoint = waypoints_list[num_pos_waypoints][1]
        goal_height_waypoint = goal_height
        #goal_height_waypoint = waypoints_list[num_pos_waypoints][2]
        goal_yaw_waypoint = np.arctan2(goal_y_waypoint , goal_x_waypoint)
        
        
        # to know where to go 
        diff_x_waypoint = goal_x_waypoint - actual_x
        diff_y_waypoint = goal_y_waypoint - actual_y
        
        diff_height_waypoint = get_difference(goal_height_waypoint, actual_height)
        diff_yaw_waypoint = get_difference(goal_yaw_waypoint, actual_yaw)
        revisar 
        
        #current_angle = HAL.getPose3d().yaw
    
    ##diff_x = objective_x_meter - x_3d
    #diff_y = objective_y_meter - y_3d
    
    #objective_angle = math.atan2(diff_y, diff_x)
    
    #angle_diff = current_angle - objective_angle
        
        #objective_angle = math.atan2(diff_y, diff_x)
    
    #angle_diff = current_angle - objective_angle
        if (diff_yaw_waypoint > math.pi):
          diff_yaw_waypoint -= 2*math.pi
        elif(diff_yaw_waypoint < -math.pi):
          diff_yaw_waypoint +=2*math.pi
        
        
        print("-------------------------------------------------------------")
        print("actual")
        print(actual_x, actual_y, actual_height, actual_yaw)
        print("goal")
        print(goal_x_waypoint, goal_y_waypoint, goal_height_waypoint, goal_yaw_waypoint)
        print("diff")
        print(diff_x_waypoint, diff_y_waypoint, diff_height_waypoint, diff_yaw_waypoint)
        
        HAL.set_cmd_mix(-1.0, 0.0, goal_height_waypoint, 0.0)
        
        """
        # significa que está alineado
        if(abs(diff_yaw_waypoint) < 0.1):
          # mover a lo largo del eje x
          if(abs(diff_x_waypoint) > abs(diff_y_waypoint)):
            if(diff_x_waypoint > 0):
      
              HAL.set_cmd_mix(0.5, 0.0, goal_height_waypoint, 0.0)
  
            else:
              HAL.set_cmd_mix(-0.5, 0.0, goal_height_waypoint, 0.0)
            
          else: 
            # mover a lo largo del eje y 
            
            if(diff_y_waypoint > 0): 
              
              HAL.set_cmd_mix(0.0, -0.5, goal_height_waypoint, 0.0)
              
            else: 
              
              HAL.set_cmd_mix(0.0, -0.5, goal_height_waypoint, 0.0)
              
    
        else: 
          #HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, 0.0)
          HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, 0.15)
        """
        
        # hemos llegado
        if(abs(diff_x_waypoint) < 0.1 and  abs(diff_y_waypoint) < 0.1 and  abs(diff_height_waypoint) < 0.1 and abs(diff_yaw_waypoint) < 0.01):
          num_pos_waypoints += 1
        
        #HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, 1.0)
        # hay que ir hacia adelante
        
  
        """
       # Navigation logic 
        if (diff_yaw_waypoint > math.pi):
          diff_yaw_waypoint -= 2*math.pi
        elif(diff_yaw_waypoint < -math.pi):
          diff_yaw_waypoint +=2*math.pi
          
          
        if(diff_yaw_waypoint > 0):
          #HAL.setW(0.5 + Kp*diff_yaw_waypoint)
          HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, 0.5 + Kp*diff_yaw_waypoint)

        else: 
          #HAL.setW(-0.5 - Kp*diff_yaw_waypoint)
          HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, -0.5 - Kp*diff_yaw_waypoint)
      
        # means that it is so close that it goes forward 
        if abs(diff_yaw_waypoint) < 0.0001:
          #HAL.setV(1.0)
          HAL.set_cmd_mix(1.0, 0.0, goal_height_waypoint, 0.0)
        else:
          #HAL.setV(0.0)
          HAL.set_cmd_mix(0.0, 0.0, goal_height_waypoint, 0.0)
      
        # has reached to destination
        if (abs(diff_x) <= 0.1 and abs(diff_y) <= 0.1):
          num_pos_waypoints += 1
          #has_reached = True
        """
        #print(HAL.get_velocity())
        #print(HAL.get_yaw_rate())
        
        #if(diff_x_waypoint > 0.1 or diff_y_waypoint > 0.1 or  diff_height_waypoint > 0.1 or diff_yaw_waypoint > 0.01):
          #HAL.set_cmd_pos(goal_x_waypoint ,goal_y_waypoint ,goal_height_waypoint , goal_yaw_waypoint)
          #vx = 0.5 * diff_x_waypoint  # Ajusta la ganancia según sea necesario
          #vy = 0.5 * diff_y_waypoint
          #vz = 0.5 * diff_height_waypoint
          #az = 0.5 * diff_yaw_waypoint
          #HAL.set_cmd_vel(vx, vy, vz, az)
          
          #HAL.set_cmd_mix(0.5, 0.0, goal_height_waypoint, 0.0)
        
        #else:
          #num_pos_waypoints += 1
          
      else: 
          phase_finding = False
        
        
        #for point in waypoints_list:
        #print(goal_x_waypoint)
        
    #  for point in waypoints_list:
    #print(point)
    
    # if robot lleva 10 minutos operando, vuelva a cargarse 
    # retome por donde se haya quedado 
      
      
 
    
    # Show ventral and frontal image
    frontal_image = HAL.get_frontal_image()
    GUI.showImage(frontal_image)
    ventral_image = HAL.get_ventral_image()
    GUI.showLeftImage(ventral_image)
    
    
  