from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
import time
import copy
import math

"""
def distancia_euclidiana(punto1, punto2):
    return np.sqrt(np.sum((np.array(punto1) - np.array(punto2))**2))

def calcular_distancias_filtrar(array_puntos, umbral):
    num_puntos = len(array_puntos)
    
    resultados = []

    for i in range(num_puntos):
        for j in range(num_puntos):
            if i != j:
                origen = array_puntos[i]
                destino = array_puntos[j]
                distancia = distancia_euclidiana(origen, destino)

                resultados.append({
                    'origen': origen,
                    'destino': destino,
                    'distancia': distancia
                })

    # Filtrar los resultados por distancia mayor de 3
    resultados_filtrados = [resultado for resultado in resultados if resultado['distancia'] > umbral]

    return resultados_filtrados
    
"""

def calcular_distancia(coord1, coord2):
    """Calcula la distancia euclidiana entre dos coordenadas."""
    return math.sqrt((int(coord1[0]) - int(coord2[0]))**2 + (int(coord1[1]) - int(coord2[1]))**2)

def agregar_coordenada(coord, lista_coordenadas):
    """Agrega una coordenada a la lista si no está dentro de un radio de 3 metros de ninguna coordenada existente."""
    for existente_coord in lista_coordenadas:
        distancia = calcular_distancia(coord, existente_coord)
        if distancia <= 3.0:
            #print(f"La coordenada {coord} está a {distancia} metros de {existente_coord}. No se agregará.")
            return lista_coordenadas
    #print(f"Agregando la coordenada {coord}.")
    lista_coordenadas.append(coord)
    return lista_coordenadas

def detect_faces(image, detector, locations): 
  
  initial_angle = 0
  incremented_angle = 20
  times_detected = 0
  #locations = []

  while initial_angle < 360:
    
    # create copy
    img_copia = copy.deepcopy(image)
  
    # Make rotation of copy image 
    rows, cols, _ = img_copia.shape
    matriz_rotacion = cv2.getRotationMatrix2D((cols / 2, rows / 2), initial_angle, 1)
    img_copia = cv2.warpAffine(img_copia, matriz_rotacion, (cols, rows))

    # convert image in grayscale
    gray_copia = cv2.cvtColor(img_copia, cv2.COLOR_BGR2GRAY)

    # make detection of faces in the image
    faces_result = detector.detectMultiScale(gray_copia)

    for (x, y, w, h) in faces_result:
        times_detected += 1

    # wait 0.5 seconds to next angle 
    cv2.waitKey(500)  
    
    # increment rotation angle 
    initial_angle += incremented_angle


  if(times_detected > 1):
    times_detected = 1
    print("DETECTADO" + str(HAL.get_position()))
    locations.append(HAL.get_position())
    # store value

  #return locations
  
def increment_x(current_point, value_for_x):
  
  point = current_point
  if(point[0] > 0):
    point[0] += value_for_x
  else:
    point[0] -= value_for_x
  
  return point
  
def decrement_x(current_point, value_for_x):
  
  point = current_point
  if(point[0] > 0):
    point[0] -= value_for_x
  else:
    point[0] += value_for_x
  
  return point
  
def increment_y(current_point, value_for_y):
  
  point = current_point
  if(point[1] > 0):
    point[1] += value_for_y
  else:
    point[1] -= value_for_y
  
  return point
  
def decrement_y(current_point, value_for_y):
  
  point = current_point
  if(point[1] > 0):
    point[1] -= value_for_y
  else:
    point[1] += value_for_y
  
  return point

def generate_rectangles(init_point, num_turns):
    waypoints = []
    
    # init point
    current_point = np.array(init_point)
    waypoints.append(tuple(current_point))
    
    for _ in range(num_turns):

        # first (x, y + [0-10])
        for i in range(10):
          current_point = increment_y(current_point, 1)
          waypoints.append(tuple(current_point))

        # second (x-2, y) 
        current_point = decrement_x(current_point, 2)
        waypoints.append(tuple(current_point))

        # third (x, y - [0 -20])
        for i in range(20):
          current_point = decrement_y(current_point, 1)
          waypoints.append(tuple(current_point))
          
        # fourth (x-2, y)
        current_point = decrement_x(current_point, 2)
        waypoints.append(tuple(current_point))

        # fifth (x, y + [0-10])
        for i in range(10):
          current_point = increment_y(current_point, 1)
          waypoints.append(tuple(current_point))

    return waypoints
    
def get_difference(objective, actual): 
    if (objective < actual): 
      dif = actual - objective
    else:
      dif = objective - actual
      
    return dif
  
actual_height = 0.0
goal_height = 3.0

# Previously converted corrdinates from GPS to UTM
east_boat_utm = 430492.2
north_boat_utm = 4459161.1

east_survivors_utm = 430532.5
north_survivors_utm = 4459131.8

# calculate distance from goal to actual
goal_x = east_survivors_utm - east_boat_utm
goal_y = north_survivors_utm - north_boat_utm
# calculate goal yaw 
goal_yaw = np.arctan2(goal_y , goal_x)

# phases
phase_take_off = True  
phase_go_to_survivors = False
phase_finding = False
phase_go_boat = False
phase_recharging = False
phase_landing = False

# creates waypoints to navigate 
num_pos_waypoints = 0
init_point = [goal_x, goal_y]
num_turns = 5
waypoints_list = generate_rectangles(init_point, num_turns)

# Create face detection 
face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# check if clasificator is loaded correctly
if face_detector.empty():
    print("Error loading clasifficator.")
    exit()

# store survivors positions 
all_survivors_positions = []
coordenadas_vistas = []
#real_survivors_positions = []

tiempo_inicio = time.time()
print("++++++++++++++++++++++++++++++++++++++++++")

while True:
 
    actual_x = HAL.get_position()[0]
    actual_y = HAL.get_position()[1]
    actual_height = HAL.get_position()[2]
    actual_yaw = HAL.get_yaw()
    
    diff_x = get_difference(goal_x, actual_x)
    diff_y = get_difference(goal_y,  actual_y)
    diff_height = get_difference(goal_height, actual_height)
    diff_yaw = get_difference(goal_yaw, actual_yaw)

    #state 1
    if(phase_take_off): 
      if(diff_height > 0.1):
        HAL.takeoff(goal_height)
      else: 
        phase_take_off = False
        phase_go_to_survivors = True
    
    #state 2
    if(phase_go_to_survivors): 
      if(diff_x > 0.1 or diff_y > 0.1 or  diff_height > 0.1 or diff_yaw > 0.01):
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_go_to_survivors = False
        phase_finding = True
        
    # state 3    
    if(phase_finding):
      if(num_pos_waypoints < len(waypoints_list)):
        
        goal_x_waypoint = waypoints_list[num_pos_waypoints][0]
        goal_y_waypoint = waypoints_list[num_pos_waypoints][1]
        goal_height_waypoint = goal_height
        goal_yaw_waypoint = np.arctan2(goal_y_waypoint , goal_x_waypoint)
        
        diff_x_waypoint = get_difference(goal_x_waypoint, actual_x)
        diff_y_waypoint = get_difference(goal_y_waypoint,  actual_y)
        diff_height_waypoint = get_difference(goal_height_waypoint, actual_height)
        diff_yaw_waypoint = get_difference(goal_yaw_waypoint, actual_yaw)
        
        ventral_image = HAL.get_ventral_image()
        
        # go to the point 
        if(diff_x_waypoint > 0.1 or diff_y_waypoint > 0.1 or  diff_height_waypoint > 0.1 or diff_yaw_waypoint > 0.01):
          HAL.set_cmd_pos(goal_x_waypoint ,goal_y_waypoint ,goal_height_waypoint , goal_yaw_waypoint)
          #all_survivors_positions = detect_faces(ventral_image, face_detector)
          
        else:
          num_pos_waypoints += 1
         
        # detect faces 
        detect_faces(ventral_image, face_detector, all_survivors_positions)
        #resultados_filtrados = calcular_distancias_filtrar(all_survivors_positions, umbral_distancia)
        GUI.showLeftImage(ventral_image) 
        
        
      else: 
          phase_finding = False
      
      
    # check baterries (has spent 10 minutes) 
    tiempo_transcurrido = time.time() - tiempo_inicio
    if(tiempo_transcurrido >= 600.00):
      #print("Come to charge")
      phase_finding = False
      phase_go_boat = True
      
    # state 4
    if(phase_go_boat):
      goal_x = 0.0
      goal_y = 0.0
      goal_yaw = np.arctan2(goal_y , goal_x)

      if(diff_x > 0.1 or diff_y > 0.1 or  diff_height > 0.1 or diff_yaw > 0.01):
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_go_boat = False
        #phase_recharging = True
        if(num_pos_waypoints < len(waypoints_list)):
          phase_recharging = True
        else: 
          phase_landing = True
          
    # state 5: simulates that it is recharging   
    if(phase_recharging):
      print("RECHARGING")
      
      #if (num_pos_waypoints < len(waypoints_list):
      #  print("I need to finish searching")
      #  tiempo_inicio = time.time()
      
      #  phase_recharging = False
      #  phase_finding = True
        
    if(phase_recharging and (num_pos_waypoints < len(waypoints_list))):
      print("I need to finish searching")
      tiempo_inicio = time.time()
      
      phase_recharging = False
      phase_finding = True
      
    if(num_pos_waypoints >= len(waypoints_list)):
      #print("FINISHED")
      phase_finding = False
      phase_go_boat = True
      
    # state 6 
    if(phase_landing):
      
      HAL.land()
      
      #umbral_distancia = 3
      #resultados_filtrados = calcular_distancias_filtrar(all_survivors_positions, umbral_distancia)

      # Imprimir los resultados filtrados
      #print("Resultados filtrados:")
      #for resultado in resultados_filtrados:
      #  print(f"Desde {resultado['origen']} hasta {resultado['destino']}: {resultado['distancia']}")
      for resultado in all_survivors_positions:
        coordenadas_vistas = agregar_coordenada(resultado, coordenadas_vistas)

      # Imprimir la lista final de coordenadas
      print("Final coordinates:")
      for coord in coordenadas_vistas:
        print(coord)
      
      phase_landing = False
      #break
    
      # print survivors locations
      
    #print(num_pos_waypoints, len(waypoints_list))
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      