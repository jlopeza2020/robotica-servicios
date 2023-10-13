from GUI import GUI
from HAL import HAL
import cv2
import numpy as np
from enum import Enum
import math
import time

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
            
            if (image[i][j] == BLACK):
              cell.occupied = True
              
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    # check if any pixel in the cell is black 
                    if(image[x][y] == BLACK):
                        is_black = True
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
    
    
def remove_duplicate_coordinates(coords):
    unique_coords = set()
    result_coords = []

    for coord in coords:
        if tuple(coord) not in unique_coords:
            result_coords.append(coord)
            unique_coords.add(tuple(coord))

    return result_coords
    
def remove_du_coordinates(array1, array2):
    # Create a set to store unique coordinates
    unique_coordinates = set()

    # Iterate through array2 and add coordinates to the set
    for coord in array2:
        unique_coordinates.add(tuple(coord))

    # Iterate through array1 and remove coordinates that are in the set
    unique_array1 = [coord for coord in array1 if tuple(coord) not in unique_coordinates]

    return unique_array1


# Example array with coordinates (x, y)
#coordinates = [(1, 2), (3, 4), (1, 2), (5, 6), (3, 4)]

# Remove duplicate coordinates
#nique_coordinates = remove_duplicate_coordinates(coordinates)


map = get_unibotics_map()
# create cell array that has the same dimension of the grid (32x32)
# to acces each position goes from 0 to 31  
cells = [[Cell(x_map, y_map) for x_map in range(COLS)] for y_map in range(ROWS)]

dilated_image = dilate_black_pixels(map)
resized_map = resize_image(dilated_image, NEW_IMG_WIDTH, NEW_IMG_HEIGHT)
filled_map = set_scenario(resized_map,cells, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)


# PRINTS: van del 0 al 31
 # y primero, x después 
celda1 = cells[4-1][3-1]
#celda1.direction=Direction.NORTE
print("x mapa: " + str(celda1.x_map) + "y mapa: " + str(celda1.y_map))
print("x gazebo: " + str(celda1.x_gazebo) + "y gazebo: " + str(celda1.y_gazebo))
print("ocupada: " + str(celda1.occupied))
print("direccion: " + str(celda1.direction))

print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
celda2 = cells[3-1][3-1]
print("x mapa: " + str(celda2.x_map) + "y mapa: " + str(celda2.y_map))
print("x gazebo: " + str(celda2.x_gazebo) + "y gazebo: " + str(celda2.y_gazebo))
print("ocupada: " + str(celda2.occupied))


# Always before iterative code
draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

#init_vel = abs(HAL.getPose3d().x)
#print(init_vel)
x = 21 
y = 19

print(cells[9][19].occupied, cells[9][20].occupied)
print(cells[10][19].occupied, cells[10][20].occupied)
print(cells[11][19].occupied, cells[11][20].occupied)

goes_west = True
goes_north = False
goes_east = False
goes_south = False 


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

for i in range (0,50):
  
  
  # guardar los puntos de retornos
  # oeste
  if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
    paint_cell(filled_map, x-1, y, CELL_WIDTH, CELL_HEIGHT, RED)
    return_points.append([y, x-1])
  
  # norte 
  if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
     paint_cell(filled_map, x, y-1, CELL_WIDTH, CELL_HEIGHT, RED)
     return_points.append([y-1, x])
  # norte 
 # cells[y-1-1][x-1]
  # este
  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
    paint_cell(filled_map, x+1, y, CELL_WIDTH, CELL_HEIGHT, RED)
    return_points.append([y, x+1])
    
  # sur 
  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
    paint_cell(filled_map, x, y+1, CELL_WIDTH, CELL_HEIGHT, RED)
    return_points.append([y+1, x])
  
  #cells[y-1][x+1-1]
  # sur 
  #cells[y+1-1][x-1]
  
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
        move_points.append([y,  x ])
        
        goes_south = False
        goes_west = True
        
        
        
print("Move"+ str(move_points))
print("Return"+ str(return_points))
# eliminate duplicates
#print(return_points)
#unique_array = remove_duplicates_2d_array(return_points)
unique_coordinates = remove_duplicate_coordinates(return_points)

# Example arrays with coordinates (x, y)
#array1 = [(1, 2), (3, 4), (5, 6), (7, 8)]
#array2 = [(3, 4), (7, 8), (5, 6), (8, 9), (1, 2)]

#print(return_points)
# Remove coordinates from array1 that are in array2
unique_array1 = remove_du_coordinates(unique_coordinates, move_points)


print(unique_array1)   

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

    #x = 21 
    #y = 19
    
    # MMEJORAR ESTOO!!!!
    # first check WEST
    #print(x, cells[y-1][x-1-1].occupied, cells[y-1][x-1-1].cleaned)
    # si la celda contigua es blanca y no ha sido barrida
    
    ## WEST PRIMERO !!!!!
    
    # primero va hacia el oeste
    #if (goes_west):
      
      # si la celda a su izq es blanca y está sucia: limpia la actual y avanza hacia la izq 
    #  if (cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
        #time.sleep(1)
    #    x = x-1
    
      # si la celda a su izq es negra y la actual está sucia: limpia la actual y cambia de dirección 
    #  if(cells[y-1][x-1-1].occupied is True and cells[y-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_west = False
    #    goes_north = True
        
        
      # si la celda a su izq es blanca pero ya está limpia: limpia la actual y cambia de dirección
    #  if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is True):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, VIOLET)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_west = False
    #    goes_north = True
        
    #if (goes_north): 
      
      # si la celda encima de ella es blanca y está sucia: limpia la actual y avanza hacia arriba
    #  if (cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
        #time.sleep(1)
    #    y = y-1
    
      # si la celda encima de ella es negra y la actual está sucia: limpia y cambia de dirección
    #  if(cells[y-1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_north = False
    #    goes_east = True
        
      # si la celda encima de ella es blanca pero está limpia: limpia la actual y cambias de dirección
    #  if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is True):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, YELLOW)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_north = False
    #    goes_east = True
        
    #if (goes_east):
      
      # si la celda de su derecha es blanca y está limpia: limpia la actual y avanza de dirección 
    #  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])

      #time.sleep(1)
    #    x = x + 1
        
      # si la celda de su derecha está ocupada pero la actual está sucia: limpia la actual y cambia de dirección
    #  if(cells[y-1][x+1-1].occupied is True and cells[y-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_east = False
    #    goes_south = True
      
      # si la celda de su dereca es blanca y estaña limpia: limpia la actual y cambia de dirección
    #  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is True):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, GREEN)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_east = False
    #    goes_south = True
        
    #if (goes_south):
      
      # si la celda  que tiene por debajo es blanca y no está limpia: limpia la actual y avanzas 
    #  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
        
      #time.sleep(1)
    #    y = y + 1
        
      # si la celda por debajo es negra y la actual está sucia: limpias la actual y cambias de dirección
    #  if(cells[y+1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
      
    #    goes_south = False
    #    goes_west = True
        
        
      # si la celda por debajo es blanca y está limpia: limpias la actual y cambias de dirección
    #  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is True):
    #    paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLUE)
    #    cells[y-1][x-1].cleaned = True
    #    move_points.append([y-1, x -1])
        
    #    goes_south = False
    #    goes_west = True
        
        
    
    #  print("si")
    #print(x, y)
    #print(move_points)
      
      
    #  paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, BLACK)
      
    draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

      
    #print()
    
    
    # Show the updated map
    #paint_cell(filled_map, x, y, CELL_WIDTH, CELL_HEIGHT, RED)
    
    GUI.showNumpy(filled_map)