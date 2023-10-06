import cv2
import numpy as np

NEW_WIDTH = 512
NEW_HEIGHT = 512

CELL_WIDTH = 16
CELL_HEIGHT = 16

def read_image(image):
    # Load the image
    map_img = cv2.imread(image)

    # Check if the image is loaded successfully
    if map_img is None:
        print('Error: Unable to load image.')
        exit()

    return map_img

def resize_image(image, new_width, new_height):
    resized_image = cv2.resize(image, (new_width, new_height))
    return resized_image

def draw_rectangles(image, rect_width, rect_height):  

    height, width, _ = image.shape

    # Iterate through the image and draw rectangles
    for i in range(0, width, rect_width):
        for j in range(0, height, rect_height):
            # Define the top-left and bottom-right corners of the rectangle
            pt1 = (i, j)
            pt2 = (i + rect_width, j + rect_height)
            # Draw green rectangle
            cv2.rectangle(image, pt1, pt2, (0, 255, 0), 2)


# dilate = erode since we want to expand black cells
def dilate_image(image):

    kernel = np.ones((5, 5), np.uint8) 

    expanded_image = cv2.erode(image, kernel, iterations=1)
    return expanded_image

def fill_black_cells(image, image_width, image_height, cell_width, cell_height):
    # pintar las celdas 

# Dimensiones de la imagen
#ancho_imagen = 512
#alto_imagen = 512

# Tamaño de los bloques
#tamano_bloque = 16

    color_negro = np.array([0, 0, 0])

    is_black = False
    for i in range(0, image_height, cell_height):
        for j in range(0, image_width, cell_width):
        # Reiniciar la bandera para cada bloque
        #hay_pixel_negro = False
        
        # Comprobar si hay algún píxel en negro dentro del bloque actual
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    if np.array_equal(color_negro,image[x][y]):
                        is_black = True

            if is_black:
                for x in range(i, i + cell_height):
                    for y in range(j, j + cell_width):

                        image[x][y] = color_negro
                is_black = False
    return image

        




# in unibotics
#map_img = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')
map = read_image('mapgrannyannie.png')
dilated_image = dilate_image(map)
# 800x800 is the new dimension
resized_map = resize_image(dilated_image, 512, 512)

filled_map = fill_black_cells(resized_map, 512, 512, 16, 16)


# pintar las celdas 

# Dimensiones de la imagen
#ancho_imagen = 512
#alto_imagen = 512

# Tamaño de los bloques
#tamano_bloque = 16

#color_negro = np.array([0, 0, 0])

#is_black = False
#for i in range(0, alto_imagen, tamano_bloque):
#    for j in range(0, ancho_imagen, tamano_bloque):
        # Reiniciar la bandera para cada bloque
        #hay_pixel_negro = False
        
        # Comprobar si hay algún píxel en negro dentro del bloque actual
#        for x in range(i, i + tamano_bloque):
#            for y in range(j, j + tamano_bloque):

#                if np.array_equal(color_negro,resized_map[x][y]):
#                    is_black = True

#        if is_black:
#            for x in range(i, i + tamano_bloque):
#                for y in range(j, j + tamano_bloque):

#                    resized_map[x][y] = color_negro
#            is_black = False

        



# 16x16 is the dimension of the cell 
# MAS PROBABLE QUE SE DIBUJE AL FINAL 
draw_rectangles(filled_map, 16, 16)

# in unibotics is showNumpy
cv2.imshow('dilated map', filled_map)
cv2.waitKey(0)
cv2.destroyAllWindows()