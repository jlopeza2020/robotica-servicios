import cv2
import numpy as np


#WIDTH_DIM = 800
#HEIGH_DIM = 800
#CELL_WIDTH_DIM = 16
#CELL_WIDTH_DIM = 16

def read_image(image):
    # unibotics
    #map_img = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')
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


    # Get the dimensions of the image
    # original size (1012,1013)
    height, width, _ = image.shape

# Iterate through the image and draw rectangles
    for i in range(0, width, rect_width):
        for j in range(0, height, rect_height):
            # Define the top-left and bottom-right corners of the rectangle
            pt1 = (i, j)
            pt2 = (i + rect_width, j + rect_height)
            # Draw green rectangle
            cv2.rectangle(image, pt1, pt2, (0, 255, 0), 2)



map = read_image('mapgrannyannie.png')
# 800x800 is the new dimension
resized_map = resize_image(map, 800, 800)
# 16x16 is the dimension of the cell 
# MAS PROBABLE QUE SE DIBUJE AL FINAL 
draw_rectangles(resized_map, 16, 16)
#dilatar la imagen 

# pintar las celdas 

# Dimensiones de la imagen
#ancho_imagen = 800
#alto_imagen = 800

# Tamaño de los bloques
#tamano_bloque = 16

# Color para los bloques (en formato BGR, es decir, Azul, Verde, Rojo)
#color_bloque = (0, 0, 255)  # Aquí, se usa verde brillante

# Crear una imagen en blanco
#imagen = np.zeros((alto_imagen, ancho_imagen, 3), dtype=np.uint8)

# Pintar bloques de 16x16 píxeles con el color especificado
#for i in range(0, alto_imagen, tamano_bloque):
#    for j in range(0, ancho_imagen, tamano_bloque):
#        resized_map[i:i+tamano_bloque, j:j+tamano_bloque] = color_bloque

# Mostrar la imagen
#cv2.imshow('Imagen con bloques de color', imagen)

#color cell



#white  = np.array([255, 255, 255])
#black = np.array([0, 0, 0])
#for i in range(0, 800, 16):
#    for j in range(0, 800, 16):
#        print(j)

        # Define the top-left and bottom-right corners of the rectangle
#        if np.array_equal(white,map[i][j]):
            #print("blanco")
#            resized_map[i][j] = 180
        

#draw_rectangles(resized_map, 16, 16)

# in unibotics is showNumpy
cv2.imshow('Image with Rectangles', resized_map)
cv2.waitKey(0)
cv2.destroyAllWindows()