import cv2
import numpy as np


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

#nueva_anchura = 300  # Cambia esto a la anchura deseada en píxeles
#nueva_altura = 200   # Cambia esto a la altura deseada en píxeles

# Redimensionar la imagen
#imagen_redimensionada = cv2.resize(map_img, (nueva_anchura, nueva_altura))


# Get the dimensions of the image
# original size (1012,1013)
    height, width, _ = image.shape

# Define the rectangle width and height
#rect_width = 16
#rect_height = 16

# Iterate through the image and draw rectangles
    for i in range(0, width, rect_width):
        for j in range(0, height, rect_height):
            # Define the top-left and bottom-right corners of the rectangle
            pt1 = (i, j)
            pt2 = (i + rect_width, j + rect_height)
            # Draw the rectangle
            cv2.rectangle(image, pt1, pt2, (0, 255, 0), 2)



map = read_image('mapgrannyannie.png')
resized_map = resize_image(map, 800, 800)
draw_rectangles(resized_map, 16, 16)

cv2.imshow('Image with Rectangles', resized_map)
cv2.waitKey(0)
cv2.destroyAllWindows()