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

#total_white_cells = 1024

class Direction(Enum):
    NORTE = "Norte"
    SUR = "Sur"
    ESTE = "Este"
    OESTE = "Oeste"
    
    def __str__(self):
        return self.value

class Cell:
    def __init__(self, x_map, y_map, x_gazebo=0, y_gazebo=0, occupied=False, cleaned=False, return_point=True, direction=None):
    #def __init__(self, x_map, y_map, occupied=False, cleaned=False):
        self.x_map = x_map
        self.y_map = y_map
        self.x_gazebo = x_gazebo
        self.y_gazebo = y_gazebo
        self.occupied = occupied  # True si está ocupada, False si no
        self.cleaned = cleaned  # True si ya ha sido barrida, False si no
        self.direction = direction  # Dirección cardinal (NORTE, SUR, ESTE, OESTE)



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
"""
def set_scenario(image, arr_cells, image_width, image_height, cell_width, cell_height, ):

    arr_cells_x = 0
    arr_cells_y = 0
     
    
    #is_black = False
    for i in range(0, image_height, cell_height):

        for j in range(0, image_width, cell_width):

        
            cell = arr_cells[arr_cells_x][arr_cells_y]
            cell.x_map = arr_cells_y
            cell.y_map = arr_cells_x
            cell.x_gazebo = j 
            cell.y_gazebo = i
            
            if (image[i][j] == BLACK):
              cell.occupied = True
              
            is_black = False
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    # check if any pixel in the cell is black 
                    if(image[x][y] == BLACK):
                        is_black = True
                        break
                        # = white_cells - 1

            # if exits one: paint in black the whole cell
            if is_black:
                for x in range(i, i + cell_height):
                    for y in range(j, j + cell_width):

                        image[x][y] = BLACK
                        

                # set that it is occupied
                #is_black = False
                
            # update value for filling class cell, y axis
            arr_cells_y += 1

        # update value for filling class cell, y axis and x axis
        arr_cells_x += 1
        arr_cells_y = 0
        
    return image
"""

# Fill cells in the image with black color if any pixel in the cell is black.
# Fill class cell

#BLACK = 0  # Constante para representar el color negro
#WHITE = 255  # Constante para representar el color blanco


def set_scenario(image, arr_cells, image_width, image_height, cell_width, cell_height, ):

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
            
            #if (image[i][j] == BLACK):
            #  cell.occupied = True
              
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    # check if any pixel in the cell is black 
                    if(image[x][y] == BLACK):
                        is_black = True
                        cell.occupied = True
                        # = white_cells - 1

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
    
    return x_2d
  
# equation obtained by linear regresion
def get_2d_y(): 
  
    y_3d = HAL.getPose3d().y
    y_2d = 13.959 + 3.092*y_3d
  
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


def store_return_points(cells, return_points, y, x, cell_width, cell_height, color):
  
  print(cells[y-1][x-1-1].occupied)
  # oeste
  if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
    paint_cell(filled_map, x-1, y, cell_width, cell_height, RED)
    return_points.append([y, x-1])
  
  print(cells[y-1-1][x-1].occupied)
  # norte 
  if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
     paint_cell(filled_map, x, y-1, cell_width, cell_height, RED)
     return_points.append([y-1, x])
   
  print(cells[y-1][x+1-1].occupied)
  # este
  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
    paint_cell(filled_map, x+1, y, cell_width, cell_height, RED)
    return_points.append([y, x+1])
    
  print(cells[y+1-1][x-1].occupied)
  # sur 
  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
    paint_cell(filled_map, x, y+1, cell_width, cell_height, RED)
    return_points.append([y+1, x])
    


# Función para calcular la distancia euclidiana entre dos puntos
def euclidean_distance(point1, point2):
    return np.linalg.norm(point1 - point2)

# Función para encontrar la mejor coordenada usando Best-First Search
def best_first_search(start, possible_solutions):
    priority_queue = PriorityQueue()
    for solution in possible_solutions:
        priority_queue.put((euclidean_distance(start, solution), tuple(solution)))  # Convertir a tupla

    best_coordinate = None
    best_distance = float('inf')

    while not priority_queue.empty():
        _, coordinate = priority_queue.get()
        coordinate = np.array(coordinate)  # Convertir nuevamente a array
        distance = euclidean_distance(start, coordinate)
        if distance < best_distance:
            best_coordinate = coordinate
            best_distance = distance

    return best_coordinate



map = get_unibotics_map()
# create cell array that has the same dimension of the grid (32x32)
# to acces each position goes from 0 to 31  
cells = [[Cell(x_map, y_map) for x_map in range(COLS)] for y_map in range(ROWS)]

dilated_image = dilate_black_pixels(map)
resized_map = resize_image(dilated_image, NEW_IMG_WIDTH, NEW_IMG_HEIGHT)
filled_map = set_scenario(resized_map,cells, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)


# PRINTS: van del 0 al 31
 # y primero, x después 
 
 #[y-1][x+1-1]

"""
x = 20
y = 11
celda1 = cells[y-1][x-1]
#celda1.direction=Direction.NORTE
print("x mapa: " + str(celda1.x_map) + "y mapa: " + str(celda1.y_map))
print("x gazebo: " + str(celda1.x_gazebo) + "y gazebo: " + str(celda1.y_gazebo))
print("ocupada: " + str(celda1.occupied))


celda2 = cells[y-1][x-1-1]
#celda2.direction=Direction.NORTE
print("x mapa: " + str(celda2.x_map) + "y mapa: " + str(celda2.y_map))
print("x gazebo: " + str(celda2.x_gazebo) + "y gazebo: " + str(celda2.y_gazebo))
print("ocupada: " + str(celda2.occupied))


celda3 = cells[y-1-1][x-1]
#celda1.direction=Direction.NORTE
print("x mapa: " + str(celda3.x_map) + "y mapa: " + str(celda3.y_map))
print("x gazebo: " + str(celda3.x_gazebo) + "y gazebo: " + str(celda3.y_gazebo))
print("ocupada: " + str(celda3.occupied))


celda4 = cells[y-1][x-1+1]
#celda1.direction=Direction.NORTE
print("x mapa: " + str(celda4.x_map) + "y mapa: " + str(celda4.y_map))
print("x gazebo: " + str(celda4.x_gazebo) + "y gazebo: " + str(celda4.y_gazebo))
print("ocupada: " + str(celda4.occupied))


celda5 = cells[y-1+1][x-1]
#celda1.direction=Direction.NORTE
print("x mapa: " + str(celda5.x_map) + "y mapa: " + str(celda5.y_map))
print("x gazebo: " + str(celda5.x_gazebo) + "y gazebo: " + str(celda5.y_gazebo))
print("ocupada: " + str(celda5.occupied))

#print("direccion: " + str(celda1.direction))
#paint_cell(filled_map, 20, 11, CELL_WIDTH, CELL_HEIGHT, YELLOW)
"""

print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")


#for i in range(0,32):
#  for j in range(0,32):
#    if(cells[j][i].occupied == True):
#      paint_cell(filled_map, i+1, j+1, CELL_WIDTH, CELL_HEIGHT, BLUE)
      #total_white_cells = total_white_cells + 1 
# Always before iterative code
#draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

#init_vel = abs(HAL.getPose3d().x)
#print(init_vel)



goes_west = True
goes_north = False
goes_east = False
goes_south = False 


# calculate all white cells in the map
total_white_cells = 0
for i in range(0,32):
  for j in range(0,32):
    if(cells[i][j].occupied == False):
      total_white_cells = total_white_cells + 1 
     
    
    
move_points = []
return_points = []


# guardar aparte puntos de retorno: e ir comprobando si ya lo tengo, quitarlo
# doble bucle for entreel array de puntos de movimiento y el de puntos de retorno 
# guardar puntos movimiento 

# while total_white_cells != 0 




is_no_return_point = False



x = 20
y = 11

#store_return_points(cells, return_points, y, x, CELL_WIDTH, CELL_HEIGHT, RED)


#while (total_white_cells > 5): 

for i in range (0,100):
  
  store_return_points(cells, return_points, y, x, CELL_WIDTH, CELL_HEIGHT, RED)

  ## MAKE SPIRAL
  
  if (goes_west):
      
      # si la celda a su izq es blanca y está sucia: limpia la actual y avanza hacia la izq 
      if (cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        #time.sleep(1)
        x = x-1
    
      # si la celda a su izq es negra y la actual está sucia: limpia la actual y cambia de dirección 
      if(cells[y-1][x-1-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
        
      # si la celda a su izq es blanca pero ya está limpia: limpia la actual y cambia de dirección
      if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
      total_white_cells = total_white_cells - 1
      
  if (goes_north): 
      
      # si la celda encima de ella es blanca y está sucia: limpia la actual y avanza hacia arriba
      if (cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        #time.sleep(1)
        y = y-1
    
      # si la celda encima de ella es negra y la actual está sucia: limpia y cambia de dirección
      if(cells[y-1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
      # si la celda encima de ella es blanca pero está limpia: limpia la actual y cambias de dirección
      if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_east):
      
      # si la celda de su derecha es blanca y está limpia: limpia la actual y avanza de dirección 
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])

      #time.sleep(1)
        x = x + 1
        
      # si la celda de su derecha está ocupada pero la actual está sucia: limpia la actual y cambia de dirección
      if(cells[y-1][x+1-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x ])
      
        goes_east = False
        goes_south = True
      
      # si la celda de su dereca es blanca y estaña limpia: limpia la actual y cambia de dirección
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_east = False
        goes_south = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_south):
      
      # si la celda  que tiene por debajo es blanca y no está limpia: limpia la actual y avanzas 
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        
      #time.sleep(1)
        y = y + 1
        
      # si la celda por debajo es negra y la actual está sucia: limpias la actual y cambias de dirección
      if(cells[y+1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_south = False
        goes_west = True
        
        
      # si la celda por debajo es blanca y está limpia: limpias la actual y cambias de dirección
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is True):
        paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
        cells[y-1][x-1].cleaned = True
        move_points.append([y,  x])
        
        goes_south = False
        goes_west = True
        
      total_white_cells = total_white_cells - 1
       
       
  # Means you have reached to a no return point
  if (cells[y-1][x-1].cleaned):
      is_no_return_point = True
      #print(y,x)
      
  if (is_no_return_point):
      
#print("Move"+ str(move_points))
#print("Return"+ str(return_points))

# add condition if it is stuck

      #print(y, x)

      unique_coordinates = remove_duplicates_from_list_of_coordinates(return_points)
      finally_return_points = remove_duplicates_from_arrays(unique_coordinates, move_points)

      #print(finally_return_points)



      current_cell = np.array([y, x])
#possible_solutions = np.array([[18, 21], [19, 22], [20, 21], [20, 20], [20, 19], [20, 18], [20, 17], [17, 21]])

      best_coordinate = best_first_search(current_cell, finally_return_points)

      #print(best_coordinate)
      
      #print(best_coordinate[0])
      
      y = best_coordinate[0]
      
      x = best_coordinate[1]
      
      is_no_return_point = False
      
  
      

 # print(total_white_cells)
#paint_cell(filled_map, 21, 17, CELL_WIDTH, CELL_HEIGHT, ORANGE)


while True:
  
    #print(total_white_cells)
    #paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, RED)
    # Assuming HAL.getPose3d().x and HAL.getPose3d().y provide the current position in the map
    #print(abs(HAL.getPose3d().x), abs(HAL.getPose3d().y))
    
    #Turn
    #grados = HAL.getPose3d().yaw * (180 / math.pi)
    #print(round(abs(grados)))
    #HAL.setW(0.1)
    
    # avanza 2 casillas
    #vel = abs(HAL.getPose3d().x)
    #if(vel > (init_vel - 0.32*2) ):
      
    #  HAL.setV(1)
    #  HAL.setW(0)
    #else: 
    #  HAL.setV(0)
    #  HAL.setW(0)
    
    #HAL.setV(1)
    #print(init_vel, vel)
    
    ###
    
    
    #current_2d_x = int(round(get_2d_x()))
    #current_2d_y = int(round(get_2d_y()))
    #print(current_2d_x, current_2d_y)
    #paint_cell(filled_map, current_2d_x, current_2d_y, CELL_WIDTH, CELL_HEIGHT, RED)

    

      
    draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

      
    
    GUI.showNumpy(filled_map)