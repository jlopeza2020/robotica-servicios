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

class Cell:
    def __init__(self, x_map, y_map, x_gazebo=0, y_gazebo=0, occupied=False, cleaned=False):

        self.x_map = x_map
        self.y_map = y_map
        self.x_gazebo = x_gazebo
        self.y_gazebo = y_gazebo
        self.occupied = occupied  # True if is occupied, False if no
        self.cleaned = cleaned  # True if is cleaned, False if no


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

# get x in cellgrid map
def get_2d_x(): 
  
    x_3d = HAL.getPose3d().x
    x_2d = 17.963 - 2.915*x_3d
    
    return x_2d
  
# get y in cellgrid map
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


def store_return_points(cells, filled_map, return_points, y, x, cell_width, cell_height, color):
  
  # WEST
  if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
    return_points.append([y, x-1])
  
  # NORTH
  if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
     return_points.append([y-1, x])
   
  # EAST
  if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):
    return_points.append([y, x+1])
    
  # SOUTH 
  if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
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


def get_y_meter(y_pixel_central):
  
  y_meter = -4.671 + 0.02*y_pixel_central

  return y_meter
  

def get_x_meter(x_pixel_central):
  
  x_meter = 5.841 - 0.021*x_pixel_central
  
  return x_meter
  
  
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

initial_2d_x = int(round(get_2d_x()))
initial_2d_y = int(round(get_2d_y()))

x = initial_2d_x
y = initial_2d_y


total_white_cells = 418

while(total_white_cells > 0):


  store_return_points(cells, filled_map, return_points, y, x, CELL_WIDTH, CELL_HEIGHT, RED)

  ## BSA ALGORITHM
  if (goes_west):
      
      # if a cell that stays on the left is white and dirty: clean the actual one and goes to the left  
      if (cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is False):
        
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        
        x = x-1
        
      # if a cell that stays on the left is black and the actual dirty: clean the actual one and change direction 
      if(cells[y-1][x-1-1].occupied is True and cells[y-1][x-1].cleaned is False):

        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
      # if a cell that stays on the left is white but it is cleaned: clean the actual one and change direction
      if(cells[y-1][x-1-1].occupied is False and cells[y-1][x-1-1].cleaned is True):

        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_west = False
        goes_north = True
        
      total_white_cells = total_white_cells - 1
      
  if (goes_north): 
      
      # if a cell that stays on the top is white and dirty: clean the actual one and goes up  
      if (cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is False):
        
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        
        y = y-1
        
      # if a cell that stays on the top is black and the actual dirty: clean the actual one and change direction 
      if(cells[y-1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
       
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
      # if a cell that stays on the top is white but it is cleaned: clean the actual one and change direction
      if(cells[y-1-1][x-1].occupied is False and cells[y-1-1][x-1].cleaned is True):
      
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_north = False
        goes_east = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_east):
      
      # if a cell that stays on the right is white and dirty: clean the actual one and goes to the right  
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is False):

        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])

        x = x + 1
        
      # if a cell that stays on the right is black and the actual dirty: clean the actual one and change direction 
      if(cells[y-1][x+1-1].occupied is True and cells[y-1][x-1].cleaned is False):
        
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_east = False
        goes_south = True
        
      # if a cell that stays on the right is white but it is cleaned: clean the actual one and change direction
      if(cells[y-1][x+1-1].occupied is False and cells[y-1][x+1-1].cleaned is True):
        
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_east = False
        goes_south = True
        
      total_white_cells = total_white_cells - 1
        
  if (goes_south):
      
      # if a cell that stays under is white and dirty: clean the actual one and goes down  
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is False):
      
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
        
        y = y + 1
        
      # if a cell that stays under is black and the actual dirty: clean the actual one and change direction 
      if(cells[y+1-1][x-1].occupied is True and cells[y-1][x-1].cleaned is False):
        
        cells[y-1][x-1].cleaned = True
        move_points.append([y, x])
      
        goes_south = False
        goes_west = True
        
      # if a cell that stays under is white but it is cleaned: clean the actual one and change direction
      if(cells[y+1-1][x-1].occupied is False and cells[y+1-1][x-1].cleaned is True):
        
        cells[y-1][x-1].cleaned = True
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
# omit first position since the initial one is the actual position
pos_move_coords = 1

has_reached = False
Kp = 1.5

paint_cell(filled_map, initial_2d_x, initial_2d_y, CELL_WIDTH, CELL_HEIGHT, DARK_VIOLET)

while True:

    # obtain the position of the robot in gazebo world in METERS 
    x_3d = HAL.getPose3d().x
    y_3d = HAL.getPose3d().y
    
    # obtain the position of the robot in map in CELLS
    # used for prints 
    current_2d_x = int(round(get_2d_x()))
    current_2d_y = int(round(get_2d_y()))
    
    
    # cell objective
    y_coord = unique_move_coordinates[pos_move_coords][0]
    x_coord = unique_move_coordinates[pos_move_coords][1]
    
    # cell objective converted into pixels
    objective_y_sup_izq_pixel = (y_coord) * CELL_HEIGHT 
    objective_x_sup_izq_pixel = (x_coord) * CELL_WIDTH
    
    # pixel objective chosen for navigation
    objective_y_central_pixel = objective_y_sup_izq_pixel + CELL_HEIGHT
    objective_x_central_pixel = objective_x_sup_izq_pixel - CELL_WIDTH
  
    # pixel objective converted into meters 
    objective_y_meter = get_y_meter(objective_y_central_pixel)
    objective_x_meter = get_x_meter(objective_x_central_pixel)
    
    
    current_angle = HAL.getPose3d().yaw
    
    diff_x = objective_x_meter - x_3d
    diff_y = objective_y_meter - y_3d
    
    objective_angle = math.atan2(diff_y, diff_x)
    
    angle_diff = current_angle - objective_angle
    
    # Navigation logic 
    if (angle_diff > math.pi):
      angle_diff -= 2*math.pi
    elif(angle_diff < -math.pi):
      angle_diff +=2*math.pi
     
    if(angle_diff > 0):
      HAL.setW(0.5 + Kp*angle_diff)
    else: 
      HAL.setW(-0.5 - Kp*angle_diff)
      
    # means that it is so close that it goes forward 
    if abs(angle_diff) < 0.1:
      HAL.setV(1.0)
    else:
      HAL.setV(0.0)
      
    if (abs(diff_x) <= 0.1 and abs(diff_y) <= 0.1):
      has_reached = True
   
    if(has_reached): 
      
      if(pos_move_coords < len(unique_move_coordinates)):
        
        pos_move_coords = pos_move_coords + 1
        has_reached = False
        
    paint_cell(filled_map, x_coord, y_coord, CELL_WIDTH, CELL_HEIGHT, DARK_VIOLET)
    draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)
    GUI.showNumpy(filled_map)