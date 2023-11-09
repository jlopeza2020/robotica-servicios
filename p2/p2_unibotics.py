from GUI import GUI
from HAL import HAL
import numpy as np

def generate_spiral_square(center, initial_side_length, num_turns, step_size):
    waypoints = []
    
    # Punto de inicio en la espiral
    current_point = np.array(center)
    
    # Generar la espiral cuadrada
    for _ in range(num_turns):
        for _ in range(4):
            # Agregar el punto actual a la lista de waypoints
            waypoints.append(tuple(current_point))
            
            # Moverse en el lado actual
            current_point[0] += initial_side_length
            waypoints.append(tuple(current_point))
            
            # Actualizar la posición para el próximo lado
            current_point[0] += initial_side_length
            current_point[1] -= step_size  # Incremento en y para formar la espiral cuadrada
    
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
center = [goal_x, goal_y, goal_height]
initial_side_length = 1
num_turns = 10
step_size = 1

# Generar los waypoints de la espiral cuadrada
waypoints_list = generate_spiral_square(center, initial_side_length, num_turns, step_size)

# Imprimir los waypoints generados

print("Waypoints:")
for point in waypoints_list:
    print(point)
    
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
      if(diff_x > 0.1 and diff_y > 0.1 and  diff_height < 0.1 and diff_yaw > 0.01):
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_go_to_survivors = False
        phase_finding = True
        
    if(phase_finding):
      if
      for point in waypoints_list:
    print(point)
    
    # if robot lleva 10 minutos operando, vuelva a cargarse 
    # retome por donde se haya quedado 
      
      
 
    
    # Show ventral and frontal image
    frontal_image = HAL.get_frontal_image()
    GUI.showImage(frontal_image)
    ventral_image = HAL.get_ventral_image()
    GUI.showLeftImage(ventral_image)