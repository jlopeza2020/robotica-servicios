from GUI import GUI
from HAL import HAL
import numpy as np
import math
import time


# Función para calcular las coordenadas de la espiral de Arquímedes creciente

def arquimedes_spiral_creciente(a, b, theta):
    r = a + b * theta  # Ajusta según sea necesario
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y
"""
# Función para calcular las coordenadas de la espiral de Arquímedes creciente
def arquimedes_spiral_creciente(a, b, c, theta):
    r = a + b * theta + c * theta**2  # Puedes ajustar el término cuadrático c según sea necesario
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

# Función para mover el dron en una espiral de Arquímedes creciente usando HAL.set_cmd_vel()
def move_drone_in_spiral_creciente(a, b, c, num_turns, velocity, angular_velocity, height):
    theta_interval = 2 * np.pi / (num_turns * 360)
    current_theta = 0

    while current_theta <= num_turns * 2 * np.pi:
        x, y = arquimedes_spiral_creciente(a, b, c, current_theta)

        # Calcular las velocidades lineales en x e y
        vx = velocity * np.cos(current_theta)
        vy = velocity * np.sin(current_theta)

        # Enviar comandos al dron
        HAL.set_cmd_mix(vx, vy, height, angular_velocity)
#HAL.set_cmd_vel(vx, vy, height, angular_velocity)

        # Imprimir información (opcional)
        print(f"Theta: {np.degrees(current_theta)}, X: {x}, Y: {y}, VX: {vx}, VY: {vy}")

        # Incrementar el ángulo para la siguiente iteración
        current_theta += theta_interval

        # Ajustar según sea necesario para controlar la velocidad de iteración
        time.sleep(0.1)

    # Detener el dron después de completar la espiral
    HAL.set_cmd_mix(0.0, 0.0, height, 0.0)
"""

"""
# Función para calcular las coordenadas de la espiral de Arquímedes creciente
def arquimedes_spiral_creciente(a, b, c, theta):
    r = a + b * theta + c * theta**2  # Ajusta el término cuadrático c según sea necesario
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

# Función para mover el dron en una espiral de Arquímedes creciente usando HAL.set_cmd_vel()
def move_drone_in_spiral_creciente(a, b, c, num_turns, velocity, angular_velocity, height):
    theta_interval = 2 * np.pi / (num_turns * 360)
    current_theta = 0

    while current_theta <= num_turns * 2 * np.pi:
        x, y = arquimedes_spiral_creciente(a, b, c, current_theta)

        # Calcular las velocidades lineales en x e y
        vx = velocity * np.cos(current_theta)
        vy = velocity * np.sin(current_theta)

        # Enviar comandos al dron
        HAL.set_cmd_mix(vx, vy, height, angular_velocity)

        # Imprimir información (opcional)
        print(f"Theta: {np.degrees(current_theta)}, X: {x}, Y: {y}, VX: {vx}, VY: {vy}")

        # Incrementar el ángulo para la siguiente iteración
        current_theta += theta_interval

        # Ajustar según sea necesario para controlar la velocidad de iteración
        time.sleep(0.1)

    # Detener el dron después de completar la espiral
    HAL.set_cmd_mix(0.0, 0.0, height, 0.0)
"""
# Parámetros de la espiral de Arquímedes
#a_param = 1.0
#b_param = 0.1
#num_turns_param = 2
#velocity_param = 1.0
#angular_velocity_param = 0.5  # Ajustar según sea necesario

# Llamar a la función para mover el dron en la espiral de Arquímedes
#move_drone_in_spiral(a_param, b_param, num_turns_param, velocity_param, angular_velocity_param)




"""
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
"""  
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


#num_pos_waypoints = 0
# Definir el centro, longitud del lado inicial, número de vueltas y tamaño del paso
#center = [goal_x, goal_y]
#num_turns = 10
#step_size = 2

# Generar los waypoints de la espiral cuadrada
#waypoints_list = generate_spiral_square(center, num_turns)
#
#Kp = 1.5
#for point in waypoints_list:
#    print(point)

# Parámetros de la espiral de Arquímedes creciente
#a_param_creciente = 20.0
#b_param_creciente = 2.0
#c_param_creciente = 1.0 # Puedes ajustar este valor según sea necesario
#num_turns_param_creciente = 10
#velocity_param_creciente = 2.0
#angular_velocity_param_creciente = 1.0  # Ajustar según sea necesario

# Parámetros de la espiral
a = 3.0
b = 1.0

# Parámetros de control del dron
height = 1.0  # Altura del dron
velocity = 3.0  # Velocidad lineal del dron
angular_velocity = 0.1  # Velocidad angular del dron

# Coordenadas iniciales del dron
#x_actual = 0.0
#y_actual = 0.0

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
      # Llamar a la función para mover el dron en la espiral de Arquímedes
      #move_drone_in_spiral(a_param, b_param, num_turns_param, velocity_param, angular_velocity_param)
      # Llamar a la función para mover el dron en la espiral de Arquímedes creciente
      #move_drone_in_spiral_creciente(a_param_creciente, b_param_creciente, c_param_creciente,
      #num_turns_param_creciente, velocity_param_creciente, 
      #                          angular_velocity_param_creciente,  goal_height)
      
      # move in spiral
      theta_inicial = np.arctan2(actual_y, actual_x)
      # Simular control de un dron a lo largo de la espiral desde la posición actual
      for theta in np.linspace(theta_inicial, theta_inicial + 4 * np.pi, 100):
          
          x, y = arquimedes_spiral_creciente(a, b, theta)

          # Calcula comandos de velocidad (simulación)
          vx = velocity * np.cos(theta)
          vy = velocity * np.sin(theta)

          # Envia comandos al dron (simulación)
          HAL.set_cmd_mix(vx, vy, height, angular_velocity)

          # Imprimir información (opcional)
          print(f"Theta: {np.degrees(theta)}, X: {x}, Y: {y}, VX: {vx}, VY: {vy}")

          # Ajusta según sea necesario para controlar la velocidad de iteración
          time.sleep(0.1)

      # Detener el dron después de completar la espiral
      HAL.set_cmd_mix(0.0, 0.0, height, 0.0)

      
      
      
    
    # if robot lleva 10 minutos operando, vuelva a cargarse 
    # retome por donde se haya quedado 
      
      
 
    
    # Show ventral and frontal image
    #frontal_image = HAL.get_frontal_image()
    #GUI.showImage(frontal_image)
    ventral_image = HAL.get_ventral_image()
    GUI.showLeftImage(ventral_image)
    
    
  