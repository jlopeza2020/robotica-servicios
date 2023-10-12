from GUI import GUI
from HAL import HAL
import cv2
import numpy as np

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


class Cell:
    def __init__(self, x_map, y_map, x_gazebo=0, y_gazebo=0, occupied=False, cleaned=False):
    #def __init__(self, x_map, y_map, occupied=False, cleaned=False):
        self.x_map = x_map
        self.y_map = y_map
        self.x_gazebo = x_gazebo
        self.y_gazebo = y_gazebo
        self.occupied = occupied  # True si está ocupada, False si no
        self.cleaned = cleaned  # True si ya ha sido barrida, False si no


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
def fill_black_cells(image, arr_cells, image_width, image_height, cell_width, cell_height):
#def fill_black_cells(image, image_width, image_height, cell_width, cell_height):

    #black = np.array([0, 0, 0])
    
    arr_cells_x = 0
    arr_cells_y = 0
    
    is_black = False
    for i in range(0, image_height, cell_height):
        #arr_cells_x += 1
        
        for j in range(0, image_width, cell_width):
            #arr_cells_x += 1
            #arr_cells_y += 1
        
            cell = arr_cells[arr_cells_x][arr_cells_y]
            cell.x_map = arr_cells_y
            cell.y_map = arr_cells_x
            cell.x_gazebo = j 
            cell.y_gazebo = i
            
            if (image[i][j] == BLACK):
              cell.occupied = True
              
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):
                  #print(image[x][y])

                    # chack if any pixel in the cell is black 
                    if(image[x][y] == BLACK):
                        is_black = True
                        #arr_cells[]
            
        

            
            
            #print(arr_cells_x, arr_cells_y)
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
    

map = get_unibotics_map()
cells = [[Cell(x_map, y_map) for x_map in range(COLS)] for y_map in range(ROWS)]

dilated_image = dilate_black_pixels(map)
resized_map = resize_image(dilated_image, NEW_IMG_WIDTH, NEW_IMG_HEIGHT)
filled_map = fill_black_cells(resized_map,cells, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)
#filled_map = fill_black_cells(resized_map, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)


# Fill class cell  

# check if are occupied 
#arr_cells_x = 0
#arr_cells_y = 0


#for i in range(0, 512, 16):

#    for j in range(0, 512, 16):

#        celda = cells[arr_cells_x][arr_cells_y]
#        celda.x_map = arr_cells_y
#        celda.y_map = arr_cells_x
#        celda.x_gazebo = j
#        celda.y_gazebo = i
#        if (filled_map[i][j] == BLACK):
#          celda.occupied = True
          
#        arr_cells_y += 1

#    arr_cells_x += 1
#    arr_cells_y = 0

# PRINTS: van del 0 al 31
 #y primero, x después 
celda1 = cells[4-1][3-1]
print("x mapa: " + str(celda1.x_map) + "y mapa: " + str(celda1.y_map))
print("x gazebo: " + str(celda1.x_gazebo) + "y gazebo: " + str(celda1.y_gazebo))
print("ocupada: " + str(celda1.occupied))
print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
celda2 = cells[3-1][3-1]
print("x mapa: " + str(celda2.x_map) + "y mapa: " + str(celda2.y_map))
print("x gazebo: " + str(celda2.x_gazebo) + "y gazebo: " + str(celda2.y_gazebo))
print("ocupada: " + str(celda2.occupied))


# Always before iterative code
draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

while True:
    # Assuming HAL.getPose3d().x and HAL.getPose3d().y provide the current position
    current_x = int(round(get_2d_x()))
    current_y = int(round(get_2d_y()))
    

    # Show the updated map
    paint_cell(filled_map, current_x, current_y, CELL_WIDTH, CELL_HEIGHT, RED)
    
    GUI.showNumpy(filled_map)

    
