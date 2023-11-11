from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
import time
import copy

def detect_faces(image, detector): 
  
  initial_angle = 0
  incremented_angle = 20

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
        print("DETECTADO" + str(HAL.get_position()))

    # wait 0.5 seconds to next angle 
    cv2.waitKey(500)  
    
    # increment rotation angle 
    initial_angle += incremented_angle

#def increment_x(value_for_x):
  
  
#def decrement_x(value_for_x):

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
#def decrement_y(value_for_y):
  
  
def generate_rectangles(init_point, num_turns):
    waypoints = []
    
    # Punto de inicio en la espiral
    current_point = np.array(init_point)
    waypoints.append(tuple(current_point))
    
    # Generar la espiral cuadrada
    for _ in range(num_turns):

        # primera (x, y + 10)

        #current_point = increment_y(current_point, 10)
        #waypoints.append(tuple(current_point))
        
        for i in range(10):
        #print(i)
          current_point = increment_y(current_point, 1)
          waypoints.append(tuple(current_point))

        # segunda(x-1, y) 
        current_point = decrement_x(current_point, 1)
        waypoints.append(tuple(current_point))

        # tercera (x, y - 10) y cuarta (x, y - 10)
        
        for i in range(20):
        #print(i)
          current_point = decrement_y(current_point, 1)
          waypoints.append(tuple(current_point))
          
        #current_point = decrement_y(current_point, 10)
        #waypoints.append(tuple(current_point))

        # cuarta (x, y - 10)
        #current_point = decrement_y(current_point, 10)
        #waypoints.append(tuple(current_point))

        # quinta (x-1, y)
        current_point = decrement_x(current_point, 1)
        waypoints.append(tuple(current_point))

        # sexta (x, y + 10)
        for i in range(10):
        #print(i)
          current_point = increment_y(current_point, 1)
          waypoints.append(tuple(current_point))
        #current_point = increment_y(current_point, 10)
        #waypoints.append(tuple(current_point))

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
goal_x = east_survivors_utm - east_boat_utm
#goal_x = east_survivors2boat

goal_y = north_survivors_utm - north_boat_utm
#goal_y = north_survivors2boat

# calculate goal yaw 
goal_yaw = np.arctan2(goal_y , goal_x)

# Must be 6
num_survivors_found = 0
total_survivors = 6

phase_take_off = True  
phase_go_to_survivors = False
phase_finding = False
phase_go_boat = False
phase_landing = False

# initialize waypoint to navigate 
num_pos_waypoints = 0
init_point = [goal_x, goal_y]
num_turns = 20
waypoints_list = generate_rectangles(init_point, num_turns)

for point in waypoints_list:
  print(point)
# Initialize face detector 
# Create face detection 
face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Verificar si el clasificador se ha cargado correctamente
if face_detector.empty():
    print("Error loading clasifficator.")
    exit()

# Cargar la imagen
#img = cv2.imread('ejemplo5.png')

# Verificar si la imagen se ha cargado correctamente
#if img is None:#
#    print("Error al cargar la imagen.")
#   exit()

# Convertir la imagen a escala de grises
#convert
#gray_original = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Definir el ángulo de rotación inicial y el incremento
#angulo_inicial = 0
#angulo_incremento = 20
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
        
        #print("-------------------------------------------------------------")
        #print("actual")
        #print(actual_x, actual_y, actual_height, actual_yaw)
        #print("goal")
        #print(goal_x_waypoint, goal_y_waypoint, goal_height_waypoint, goal_yaw_waypoint)
        #print("diff")
        #print(diff_x_waypoint, diff_y_waypoint, diff_height_waypoint, diff_yaw_waypoint)
        
        # go to the point 
        if(diff_x_waypoint > 0.1 or diff_y_waypoint > 0.1 or  diff_height_waypoint > 0.1 or diff_yaw_waypoint > 0.01):
          HAL.set_cmd_pos(goal_x_waypoint ,goal_y_waypoint ,goal_height_waypoint , goal_yaw_waypoint)
          detect_faces(ventral_image, face_detector)
          
        else:
          num_pos_waypoints += 1
         
        # detect faces 
        #ventral_image = HAL.get_ventral_image()
        detect_faces(ventral_image, face_detector)
        GUI.showLeftImage(ventral_image) 
        
        
      else: 
          phase_finding = False
      
      
    # si han pasado 10 minutos 
    #tiempo_inicio = time.time()
    tiempo_transcurrido = time.time() - tiempo_inicio
    if(tiempo_transcurrido >= 600.00):
      print("Come to charge")
      phase_finding = False
      #phase_go_to_survivors = False
      #phase_take_off = False
      
      phase_go_boat = True
      
      #tiempo_inicio = 0.0
      
    if(phase_go_boat):
      goal_x = 0.0
      goal_y = 0.0
      goal_yaw = np.arctan2(goal_y , goal_x)

      if(diff_x > 0.1 or diff_y > 0.1 or  diff_height > 0.1 or diff_yaw > 0.01):
        HAL.set_cmd_pos(goal_x,goal_y,goal_height, goal_yaw)
      else: 
        phase_go_boat = False
        phase_landing = True
      
      
    if(phase_landing):
      HAL.land()
      
    if(phase_landing and (num_pos_waypoints < len(waypoints_list))):
      print("I need to finish searching")
      goal_height = 2.5
      # restart timer
      tiempo_inicio = time.time()
      
      phase_landing = False
      #HAL.takeoff(goal_height)
      phase_finding = True
      
    print(tiempo_transcurrido)
    
    # Show ventral and frontal image
    #frontal_image = HAL.get_frontal_image()
    #GUI.showImage(frontal_image)
    #ventral_image = HAL.get_ventral_image()
    #GUI.showLeftImage(ventral_image)