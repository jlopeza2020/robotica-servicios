from GUI import GUI
from HAL import HAL
import cv2
import numpy as np
from enum import Enum
import math
import time
from queue import PriorityQueue

NEW_IMG_WIDTH = 512
NEW_IMG_HEIGHT = 512  

CELL_WIDTH = 16
CELL_HEIGHT = 16

ROWS = 32
COLS = 32

BLACK = 0
WHITE = 127
RED = 128
ORANGE = 129
YELLOW = 130
GREEN = 131
BLUE = 132
VIOLET = 134
DARK_VIOLET = 133


#WEST_DIR = 0
#NORTH_DIR = 90
#EAST_DIR = 180
#SOUTH_DIR = 270

"""
class Direction(Enum):
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"
    
    def __str__(self):
        return self.value
"""
class Cell:
    def __init__(self, x_map, y_map, x_gazebo=0, y_gazebo=0, occupied=False, cleaned=False):
    #def __init__(self, x_map, y_map, x_gazebo=0, y_gazebo=0, occupied=False, cleaned=False, direction=None):

    #def __init__(self, x_map, y_map, occupied=False, cleaned=False):
        self.x_map = x_map
        self.y_map = y_map
        self.x_gazebo = x_gazebo
        self.y_gazebo = y_gazebo
        self.occupied = occupied  # True si está ocupada, False si no
        self.cleaned = cleaned  # True si ya ha sido barrida, False si no
        #self.direction = direction  # Dirección cardinal (NORTE, SUR, ESTE, OESTE)


def get_unibotics_map():
    # Get map and show png map
    rgba_image = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')
    rgba_np = np.array(rgba_image)
    # Convert RGBA to RGB by removing the alpha channel
    rgb_image = rgba_np[:, :, :3]  # Keep only the first 3 channels (R, G, B)
    # Scale float values to integers in the range [0, 255]
    rgb_int = (rgb_image * 255).astype(np.uint8)
    black = np.array([0, 0, 0])
    # create a map of white  pixels
    matrix = np.random.randint(127, 128, (rgb_int.shape[0], rgb_int.shape[1]), dtype=np.uint8)

    for i in range(0,rgb_int.shape[0]):
      for j in range(0, rgb_int.shape[1]):

        if np.array_equal(black,rgb_int[i][j]):
          matrix[i][j] = BLACK
          
    return matrix


def resize_image(image, new_width, new_height):
    resized_image = cv2.resize(image, (new_width, new_height))
    return resized_image

def draw_rectangles(image, rect_width, rect_height):  

    height, width = image.shape

    # Iterate through the image and draw rectangles
    for i in range(0, width, rect_width):
        for j in range(0, height, rect_height):
            # Define the top-left and bottom-right corners of the rectangle
            pt1 = (i, j)
            pt2 = (i + rect_width, j + rect_height)
            # Draw green rectangle
            cv2.rectangle(image, pt1, pt2, (ORANGE, ORANGE, ORANGE), 2)


# dilate = erode since we want to expand black cells
def dilate_black_pixels(image):

    kernel = np.ones((5, 5), np.uint8) 

    expanded_image = cv2.erode(image, kernel, iterations=1)
    return expanded_image


# Fill cells in the image with black color if any pixel in the cell is black.
# Fill class cell
def set_scenario(image, arr_cells, image_width, image_height, cell_width, cell_height):

    arr_cells_x = 0
    arr_cells_y = 0
     
    
    is_black = False
    for i in range(0, image_height, cell_height):

        for j in range(0, image_width, cell_width):

        
            cell = arr_cells[arr_cells_x][arr_cells_y]
            cell.x_map = arr_cells_y
            cell.y_map = arr_cells_x
            cell.x_gazebo = j 
            cell.y_gazebo = i
            
            
              
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    # check if any pixel in the cell is black 
                    if(image[x][y] == BLACK):
                        is_black = True
                        cell.occupied = True

            # if exits one: paint in black the whole cell
            if is_black:
                for x in range(i, i + cell_height):
                    for y in range(j, j + cell_width):

                        image[x][y] = BLACK
                        

                # set that it is occupied
                is_black = False
                
            # update value for filling class cell, y axis
            arr_cells_y += 1

        # update value for filling class cell, y axis and x axis
        arr_cells_x += 1
        arr_cells_y = 0
        
    return image

# equation obtained by linear regresion
def get_2d_x(): 
  
    x_3d = HAL.getPose3d().x
    x_2d = 17.963 - 2.915*x_3d
    #x_2d = 17.61 - 3.13*x_3d
    return x_2d
  
# equation obtained by linear regresion
def get_2d_y(): 
  
    y_3d = HAL.getPose3d().y
    y_2d = 13.959 + 3.092*y_3d
    #y_2d = 12.72 + 3.12*y_3d
    return y_2d

def paint_cell(cell_map, x, y, cell_width, cell_height, color):
  
    for aux_y in range((y * cell_height) - cell_height, y * cell_height):
      for aux_x in range((x * cell_width) - cell_width, x * cell_width):
          cell_map[aux_y][aux_x] = color

    
def remove_duplicates_from_list_of_coordinates(coords):
    unique_coords = set()
    result_coords = []

    for coord in coords:
        if tuple(coord) not in unique_coords:
            result_coords.append(coord)
            unique_coords.add(tuple(coord))

    return result_coords
    
def remove_duplicates_from_arrays(array1, array2):
    # Create a set to store unique coordinates
    unique_coordinates = set()

    # Iterate through array2 and add coordinates to the set
    for coord in array2:
        unique_coordinates.add(tuple(coord))

    # Iterate through array1 and remove coordinates that are in the set
    unique_array1 = [coord for coord in array1 if tuple(coord) not in unique_coordinates]

    return unique_array1


def store_return_points(cells, filled_map, return_points, y, x, cell_width, cell_height, color):
  
  # WEST
  if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
    paint_cell(filled_map, x-1, y, cell_width, cell_height, RED)
    return_points.append([y, x-1])
  
  # NORTH
  if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
     paint_cell(filled_map, x, y-1, cell_width, cell_height, RED)
     return_points.append([y-1, x])
   
  # EAST
  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
    paint_cell(filled_map, x+1, y, cell_width, cell_height, RED)
    return_points.append([y, x+1])
    
  # SOUTH 
  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
    paint_cell(filled_map, x, y+1, cell_width, cell_height, RED)
    return_points.append([y+1, x])
    

#  Heuristic: Calculate eclidian distance
def euclidean_distance(point1, point2):
    return np.linalg.norm(point1 - point2)

# To calculate te best coordinate 
def best_first_search(start, possible_solutions):
    priority_queue = PriorityQueue()
    for solution in possible_solutions:
        priority_queue.put((euclidean_distance(start, solution), tuple(solution)))  # Convert int a tuple 

    best_coordinate = None
    best_distance = float('inf')

    while not priority_queue.empty():
        _, coordinate = priority_queue.get()
        coordinate = np.array(coordinate)  # Convert into and array
        distance = euclidean_distance(start, coordinate)
        if distance < best_distance:
            best_coordinate = coordinate
            best_distance = distance

    return best_coordinate


"""
def celda_a_pixel(celda_x, celda_y, ancho_celda, alto_celda):
    pixel_x = celda_x * ancho_celda + ancho_celda / 2
    pixel_y = celda_y * alto_celda + alto_celda / 2
    return (pixel_y, pixel_x)

def pixel_a_coordenada_mundo(pixel_x, pixel_y):
    mundo_x = pixel_x * 0.02 # Supongamos que 1 pixel equivale a 1/50 unidad en el mundo
    mundo_y = pixel_y * 0.02
    return (mundo_y, mundo_x)
"""    
"""   
def calcular_arcotangente(punto1, punto2):
    # Puntos representados como tuplas (x, y)
    x1, y1 = punto1
    x2, y2 = punto2
    
    # Calcula la diferencia en coordenadas x e y
    delta_x = x2 - x1
    delta_y = y2 - y1
    
    # Calcula la arcotangente en radianes
    arcotangente_rad = math.atan2(delta_y, delta_x)
    
    # Convierte la arcotangente a grados
    arcotangente_grados = math.degrees(arcotangente_rad)
    
    return arcotangente_rad, arcotangente_grados


"""
"""
def get_difference(objective, actual): 
  
    if (objective < actual): 
      dif = actual - objective
    else:
      dif = objective - actual
      
    return dif
"""  
  
def get_y_meter(y_pixel_central):
  
  y_meter = -4.671 + 0.02*y_pixel_central
  #y_meter = -4.28 + 0.01*y_pixel_central
  
  return y_meter
  

def get_x_meter(x_pixel_central):
  
  x_meter = 5.841 - 0.021*x_pixel_central
  #x_meter = 5.84 - 0.01*x_pixel_central

  
  return x_meter
  
  
def parse_laser_data(laser_data, angle_min, angle_max):
    laser = []
    for i in range(180):
        angle = math.radians(i)
        if angle_min <= angle <= angle_max:
            dist = laser_data.values[i]
            #laser.append((dist, angle))
            laser.append(dist)
    
    
    laser_mean = np.mean(laser)
    return laser_mean
    
    

map = get_unibotics_map()
# create cell array that has the same dimension of the grid (32x32)
# to acces each position goes from 0 to 31  
cells = [[Cell(x_map, y_map) for x_map in range(COLS)] for y_map in range(ROWS)]

dilated_image = dilate_black_pixels(map)
resized_map = resize_image(dilated_image, NEW_IMG_WIDTH, NEW_IMG_HEIGHT)
filled_map = set_scenario(resized_map,cells, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)


print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

goes_west = True
goes_north = False
goes_east = False
goes_south = False 


move_points = []
return_points = []

is_no_return_point = False

#x = 21
#y = 19

initial_2d_x = int(round(get_2d_x()))
initial_2d_y = int(round(get_2d_y()))

#paint_cell(filled_map, initial_2d_x + 1, initial_2d_y, CELL_WIDTH, CELL_HEIGHT, 133)

x = initial_2d_x
y = initial_2d_y


total_white_cells = 418

while(total_white_cells > 0):


  store_return_points(cells, filled_map, return_points, y, x, CELL_WIDTH, CELL_HEIGHT, RED)

  ## MAKE SPIRAL
  
  if (goes_west):
      
      # si la celda a su izq es blanca y está sucia: limpia la actual y avanza hacia la izq 
      if (cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.WEST
        
        move_points.append([y, x])
        x = x-1
        
      # si la celda a su izq es negra y la actual está sucia: limpia la actual y cambia de dirección 
      if(cells[y-1][x-1-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.WEST
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
      # si la celda a su izq es blanca pero ya está limpia: limpia la actual y cambia de dirección
      if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.WEST
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
      total_white_cells = total_white_cells - 1
      
  if (goes_north): 
      
      # si la celda encima de ella es blanca y está sucia: limpia la actual y avanza hacia arriba
      if (cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.NORTH
        move_points.append([y, x])
        y = y-1
        

      # si la celda encima de ella es negra y la actual está sucia: limpia y cambia de dirección
      if(cells[y-1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.NORTH

        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
    
        
      # si la celda encima de ella es blanca pero está limpia: limpia la actual y cambias de dirección
      if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.NORTH
        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_east):
      
      # si la celda de su derecha es blanca y está limpia: limpia la actual y avanza de dirección 
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.EAST

        move_points.append([y, x])

        x = x + 1
        

      # si la celda de su derecha está ocupada pero la actual está sucia: limpia la actual y cambia de dirección
      if(cells[y-1][x+1-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.EAST

        move_points.append([y, x])
      
        goes_east = False
        goes_south = True
        

      # si la celda de su dereca es blanca y estaña limpia: limpia la actual y cambia de dirección
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.EAST

        move_points.append([y, x])
      
        goes_east = False
        goes_south = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_south):
      
      # si la celda  que tiene por debajo es blanca y no está limpia: limpia la actual y avanzas 
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.SOUTH
        move_points.append([y, x])
        
        y = y + 1
        

      # si la celda por debajo es negra y la actual está sucia: limpias la actual y cambias de dirección
      if(cells[y+1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.SOUTH
        move_points.append([y, x])
      
        goes_south = False
        goes_west = True
        

        
      # si la celda por debajo es blanca y está limpia: limpias la actual y cambias de dirección
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        #cells[y-1][x-1].direction = Direction.SOUTH
        move_points.append([y,  x])
        
        goes_south = False
        goes_west = True
   
        
      total_white_cells = total_white_cells - 1

  
  # Means you have reached to a no return point
  if (cells[y-1][x-1].cleaned):
      is_no_return_point = True

  if (is_no_return_point):
      
      unique_return_coordinates = remove_duplicates_from_list_of_coordinates(return_points)
      finally_return_points = remove_duplicates_from_arrays(unique_return_coordinates, move_points)

      current_cell = np.array([y, x])
      best_coordinate = best_first_search(current_cell, finally_return_points)

      
      y = best_coordinate[0]
      x = best_coordinate[1]
      
      is_no_return_point = False
      


unique_move_coordinates = remove_duplicates_from_list_of_coordinates(move_points)  
# la primera la omitimos, apunta al array donde está las posiciones para moverse 
# omit first position since it is the initial one 
pos_move_coords = 1

#init_pos = HAL.getPose3d().x

# diferencia entre las casillas centrales del robot 
#dif_y = 0.0
#dif_x = 0.0

#dif_degrees = 0
#dif_rad = 0.0
#dif_x_bt = 0

#avanza_casilla =  True

# a la siguietne casilla 
has_reached = False
#arc_rad_robot = 0.0
#orientation_converted = 0.0

Kp = 1.5

#times_obstacle = 0
"""

for i in range(0, len(unique_move_coordinates)):
  
    y_coord = unique_move_coordinates[pos_move_coords][0]
    x_coord = unique_move_coordinates[pos_move_coords][1]
    
    print("Y CELDA OBJETIVO" + str(y_coord), "X CELDA OBJETIVO" + str(x_coord))
    
    # de esas celdas objetivo, calculamos la posición superior izq en píxeles 
    objective_y_sup_izq_pixel = (y_coord) * 16
    objective_x_sup_izq_pixel = (x_coord) * 16

    # calculamos el pixel central objetivo
    objective_y_central_pixel = objective_y_sup_izq_pixel + 8
    objective_x_central_pixel = objective_x_sup_izq_pixel + 8
    
    
    #print(objective_y_central_pixel, objective_x_central_pixel)
    

    objective_y_meter = get_y_meter(objective_y_central_pixel)
    objective_x_meter = get_x_meter(objective_x_central_pixel)
    
    
    print("Y METROS OBJETIVO" + str(objective_y_meter), "X METROS OBJETIVO" + str(objective_x_meter))
    
    pos_move_coords += 1
"""  
paint_cell(filled_map, initial_2d_x, initial_2d_y, CELL_WIDTH, CELL_HEIGHT, 133)

while True:
    
    
  
    
    # MODIFICAR 
    
    
    # obtenemos las posiciones del mundo de gazebo EN METROS ACTUALES 
    x_3d = HAL.getPose3d().x
    y_3d = HAL.getPose3d().y
    
    #print("Y METROS ACTUAL :"+str(y_3d), "X METROS ACTUAL :"+str(x_3d))
    
    # use it  to draw cell
    
    # obtenemos las posiciones en celdas de nuestro robot en el mapa 
    current_2d_x = int(round(get_2d_x()))
    current_2d_y = int(round(get_2d_y()))
    
    print("Y CELDA ACTUAL " + str(current_2d_y), "X CELDA ACTUAL "+ str(current_2d_x))
    
    
    # CELDA OBJETIVO
    y_coord = unique_move_coordinates[pos_move_coords][0]
    x_coord = unique_move_coordinates[pos_move_coords][1]
    
    print("Y CELDA OBJETIVO  " + str(y_coord), "X CELDA OBJETIVO  " + str(x_coord))

    
    # de esas celdas objetivo, calculamos la posición superior izq en píxeles 
    #objective_y_sup_izq_pixel = (y_coord -1) * 16
    #objective_x_sup_izq_pixel = (x_coord -1) * 16

    objective_y_sup_izq_pixel = (y_coord) * 16
    objective_x_sup_izq_pixel = (x_coord) * 16
    
    
    #print("CELL Y" + str(cells[y_coord-1][x_coord-1].y_gazebo),  "CELL X" +str(cells[y_coord-1][x_coord-1].x_gazebo))
    #print("SUP IZQ Y" + str(objective_y_sup_izq_pixel),  "SUP IZQ X" +str(objective_x_sup_izq_pixel))
    
    # calculamos el pixel central objetivo
    #objective_y_central_pixel = objective_y_sup_izq_pixel + 8
    objective_y_central_pixel = objective_y_sup_izq_pixel + 16 #working version
    #objective_y_central_pixel = objective_y_sup_izq_pixel + 8
    #objective_y_central_pixel = objective_y_sup_izq_pixel + 14
    #objective_x_central_pixel = objective_x_sup_izq_pixel - 6
    #objective_x_central_pixel = objective_x_sup_izq_pixel - 8 
    objective_x_central_pixel = objective_x_sup_izq_pixel - 16# working version
    #objective_x_central_pixel = objective_x_sup_izq_pixel + 8 
    
    
    #print(objective_y_central_pixel, objective_x_central_pixel)
    

    objective_y_meter = get_y_meter(objective_y_central_pixel)
    objective_x_meter = get_x_meter(objective_x_central_pixel)
    
    
    #print("Y METROS OBJETIVO" + str(objective_y_meter), "X METROS OBJETIVO" + str(objective_x_meter))
    
    current_angle = HAL.getPose3d().yaw
    
    error_x = objective_x_meter - x_3d
    error_y = objective_y_meter - y_3d
    
    #print(" error x" + str(error_x), "error y " + str(error_y))
    
    goal_angle = math.atan2(error_y, error_x)
    
    #print("goal angle" + str(goal_angle))
    #print("current  angle" + str(current_angle))
    
    angle_diff = current_angle - goal_angle
    
    #print(angle_diff)
    
    
    
    if (angle_diff > math.pi):
      angle_diff -= 2*math.pi
    elif(angle_diff < -math.pi):
      angle_diff +=2*math.pi
     
      
    #print(angle_diff)
    
     
    
    if(angle_diff > 0):
      HAL.setW(0.5 + Kp*angle_diff)
    else: 
      HAL.setW(-0.5 - Kp*angle_diff)
      #print(angle_diff)
      
    # significa que estoy lo suficientemente alineado y avanzo 
    if abs(angle_diff) < 0.1:
      HAL.setV(1.0)
    else:
      HAL.setV(0.0)
      
    
    
    #laser_dt = HAL.getLaserData()
    # mean of the values of the center 
    #laser_info_mean = parse_laser_data(laser_dt, 0*math.pi/180 , 180*math.pi/180)

    
    
    #print("MEANNN" + str(laser_info_mean))
  
    
    #if (round(y_3d,4) == round(objective_y_meter,4) and round(x_3d,4) == round(objective_x_meter,4)):
    
    #if (error_x <= 0.001 and error_y <= 0.001 and abs(angle_diff) >=  0.1):

    if (abs(error_x) <= 0.1 and abs(error_y) <= 0.1):
      has_reached = True
      
    ## modificar para que solo aumente uno 
    
    #laser_info_mean = 1
    #times_obstacle = 0
    
    
    #if(laser_info_mean <= 0.5):
      #has_reached = True
    #  HAL.setW(5.0)
    #  times_obstacle += 1
      #paint_cell(filled_map, current_2d_x+1, current_2d_y, CELL_WIDTH, CELL_HEIGHT, 133)
      
    #  if(times_obstacle >= 1):
    #    times_obstacle = 1
        
    #print(times_obstacle)
    
    #laser_info_mean = 1    
     

    
    #if(has_reached or times_obstacle == 1): 
    if(has_reached): 
      
      if(pos_move_coords < len(unique_move_coordinates)):
        #print(x_coord == x_3d and y_coord == y_3d, x_coord == x_3d or y_coord == y_3d)
        #if(x_coord == x_3d and y_coord == y_3d):
        
         # paint_cell(filled_map, current_2d_x , current_2d_y, CELL_WIDTH, CELL_HEIGHT, 133)
        #elif (x_coord == x_3d or y_coord == y_3d):
          #paint_cell(filled_map, current_2d_x +1 , current_2d_y, CELL_WIDTH, CELL_HEIGHT, 133)
        
        
        pos_move_coords = pos_move_coords + 1
        has_reached = False
        
      
        
      #times_obstacle = 0
        
      #dif_x = 0.0
      #dif_y = 0.0
      #dif_rad = 0.0
    
    
    paint_cell(filled_map, x_coord, y_coord, CELL_WIDTH, CELL_HEIGHT, 133)
    
    GUI.showNumpy(filled_map)
    draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)