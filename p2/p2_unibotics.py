from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
import time
import copy
"""
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
"""
def detect_faces(image, detector): 
  
  angulo_inicial = 0
  angulo_incremento = 20

  # Iterar hasta alcanzar 360 grados
  while angulo_inicial < 360:
    # Crear una copia de la imagen original
    img_copia = copy.deepcopy(image)

    # Realizar la rotación en la imagen copia
    rows, cols, _ = img_copia.shape
    matriz_rotacion = cv2.getRotationMatrix2D((cols / 2, rows / 2), angulo_inicial, 1)
    img_copia = cv2.warpAffine(img_copia, matriz_rotacion, (cols, rows))

    # Convertir la imagen copia a escala de grises
    gray_copia = cv2.cvtColor(img_copia, cv2.COLOR_BGR2GRAY)

    # Realizar la detección de caras en la imagen rotada
    faces_result = detector.detectMultiScale(gray_copia)

    # Dibujar rectángulos alrededor de las caras detectadas
    for (x, y, w, h) in faces_result:
        #img_copia = cv2.rectangle(img_copia, (x, y), (x+w, y+h), (255, 0, 0), 2)
        print("DETECTADO")

    # Mostrar la imagen con las caras detectadas
    #cv2.imshow('img', img_copia)
    cv2.waitKey(500)  # Esperar 0.5 segundos antes de pasar al siguiente ángulo

    # Incrementar el ángulo de rotación
    angulo_inicial += angulo_incremento

# Cerrar todas las ventanas al finalizar
#cv2.destroyAllWindows()

  
def generate_rectangles(init_point, num_turns):
    waypoints = []
    
    # Punto de inicio en la espiral
    current_point = np.array(init_point)
    waypoints.append(tuple(current_point))
    
    # Generar la espiral cuadrada
    for _ in range(num_turns):

        # primera (x, y + 15)
        if(current_point[1] > 0):
          current_point[1] += 10
        else:
          current_point[1] -= 10
        waypoints.append(tuple(current_point))

        # segunda(x-1, y) 
        if(current_point[0] > 0):
          current_point[0] -= 1
        else:
          current_point[0] += 1
        waypoints.append(tuple(current_point))


        # tercera (x, y - 15)
        if(current_point[1] > 0):
          current_point[1] -= 10
        else: 
          current_point[1] += 10
        waypoints.append(tuple(current_point))

        # cuarta (x, y - 51)
        if(current_point[1] > 0):
          current_point[1] -= 10
        else: 
          current_point[1] += 10
        waypoints.append(tuple(current_point))

        # quinta (x-1, y)
        if(current_point[0] > 0):
          current_point[0] -= 1
        else:
          current_point[0] += 1
        waypoints.append(tuple(current_point))

        # sexta (x, y + 15)
        if(current_point[1] > 0):
          current_point[1] += 10
        else:
          current_point[1] -= 10
        waypoints.append(tuple(current_point))

    return waypoints
    
    
def generate_spiral_square(center, num_turns, step_size):
    waypoints = []
    
    # Punto de inicio en la espiral
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
              
        # cuarta(x+4, y)
        if(current_point[0] > 0):
          current_point[0] += incremento

        else:
          current_point[0] += incremento
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

# initialize waypoint to navigate 
num_pos_waypoints = 0
init_point = [goal_x, goal_y]
num_turns = 20
waypoints_list = generate_rectangles(init_point, num_turns)


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
        
        #print("-------------------------------------------------------------")
        #print("actual")
        #print(actual_x, actual_y, actual_height, actual_yaw)
        #print("goal")
        #print(goal_x_waypoint, goal_y_waypoint, goal_height_waypoint, goal_yaw_waypoint)
        #print("diff")
        #print(diff_x_waypoint, diff_y_waypoint, diff_height_waypoint, diff_yaw_waypoint)
        
        if(diff_x_waypoint > 0.1 or diff_y_waypoint > 0.1 or  diff_height_waypoint > 0.1 or diff_yaw_waypoint > 0.01):
          HAL.set_cmd_pos(goal_x_waypoint ,goal_y_waypoint ,goal_height_waypoint , goal_yaw_waypoint)
        else:
          num_pos_waypoints += 1
         
        # detect faces 
        ventral_image = HAL.get_ventral_image()
        detect_faces(ventral_image, face_detector)
        GUI.showLeftImage(ventral_image) 
        
        
      else: 
          phase_finding = False
      
    # Show ventral and frontal image
    frontal_image = HAL.get_frontal_image()
    GUI.showImage(frontal_image)
    #ventral_image = HAL.get_ventral_image()
    #GUI.showLeftImage(ventral_image)