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


# in the end is erode the map since it expands the values 
# of the dark pixels 
def dilate_image(image):
    # dilata la zona negra 
    #kernel=np.zeros((5,5),np.uint8)
    #dilation_op= cv2.dilate(image,kernel,iterations=-3)
    #return dilation_op


    # Invertir la imagen (para que los obstáculos sean blancos)
    #imagen_invertida = cv2.bitwise_not(image)

    # Definir un kernel para la operación de erosión
    kernel = np.ones((5, 5), np.uint8)  # Kernel de 15x15 con todos los elementos como 1

    # Aplicar la erosión
    imagen_engordada = cv2.erode(image, kernel, iterations=1)

    # Invertir nuevamente para obtener los obstáculos engordados (negros)
    #imagen_engordada = cv2.bitwise_not(imagen_engordada)

    return imagen_engordada



map = read_image('mapgrannyannie.png')
dilated_image = dilate_image(map)
# 800x800 is the new dimension
resized_map = resize_image(dilated_image, 512, 512)
# 16x16 is the dimension of the cell 
# MAS PROBABLE QUE SE DIBUJE AL FINAL 
#draw_rectangles(resized_map, 16, 16)




# pintar las celdas 

# Dimensiones de la imagen
ancho_imagen = 512
alto_imagen = 512

# Tamaño de los bloques
tamano_bloque = 16


    # white color 
    #black = [255, 255, 255]
    # black color 
    #black = [0, 0, 0]
    
    
    #white  = np.array([255, 255, 255])
    #black = np.array([0, 0, 0])
#print(np.array_equal(a1,a1))
#print(np.array_equal(a1,a2))
    
    #for i in range (width):
    #  for j in range (height):
    #    if np.array_equal(white,map_img[i][j]):
        
# Color para los bloques (en formato BGR, es decir, Azul, Verde, Rojo)
#color_bloque = (0, 0, 255)  # Aquí, se usa rojo 
#color_negro = (0, 0, 0)
color_bloque  = np.array([0, 0, 255])
color_negro = np.array([0, 0, 0])
# Crear una imagen en blanco
#imagen = np.zeros((alto_imagen, ancho_imagen, 3), dtype=np.uint8)

# Pintar bloques de 16x16 píxeles con el color especificado
#for i in range(0, alto_imagen, tamano_bloque):
#    for j in range(0, ancho_imagen, tamano_bloque): 
#        resized_map[i:i+tamano_bloque, j:j+tamano_bloque] = color_bloque


# Suponiendo que 'resized_map' es una matriz que representa la imagen y tiene valores de píxeles (por ejemplo, 0 para negro)
# 'alto_imagen', 'ancho_imagen' son las dimensiones de la imagen
# 'tamano_bloque' es el tamaño del bloque (16 en este caso)
# 'color_bloque' es el color del bloque

#hay_pixel_negro = False  # Variable para indicar si hay algún píxel en negro en el bloque

for i in range(0, alto_imagen, tamano_bloque):
    for j in range(0, ancho_imagen, tamano_bloque):
        # Reiniciar la bandera para cada bloque
        #hay_pixel_negro = False
        
        # Comprobar si hay algún píxel en negro dentro del bloque actual
        for x in range(i, i + tamano_bloque):
            for y in range(j, j + tamano_bloque):
                #if resized_map[x, y] == color_negro:  # Comprobar si el píxel es negro
                if np.array_equal(color_negro,resized_map[x][y]):
                    resized_map[x][y] = color_negro
                    #hay_pixel_negro = True
                    #break  # Si se encuentra un píxel en negro, salir del bucle interior
                #resized_map[x][y] = color_bloque

            #if hay_pixel_negro:
            #    break  # Si se encontró un píxel en negro, salir del bucle exterior
                
        # Si hay al menos un píxel en negro, imprimir un mensaje
        #if hay_pixel_negro:
        #    print("Hay al menos un píxel en negro en el bloque en la posición ({}, {})".format(i, j))
        #else:
        #    print("No hay píxeles en negro en el bloque en la posición ({}, {})".format(i, j))

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
cv2.imshow('dilated map', resized_map)
cv2.waitKey(0)
cv2.destroyAllWindows()