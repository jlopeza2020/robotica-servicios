import cv2
import numpy as np

NEW_IMG_WIDTH = 512
NEW_IMG_HEIGHT = 512

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
def dilate_black_pixels(image):

    kernel = np.ones((5, 5), np.uint8) 

    expanded_image = cv2.erode(image, kernel, iterations=1)
    return expanded_image


# Fill cells in the image with black color if any pixel in the cell is black.
def fill_black_cells(image, image_width, image_height, cell_width, cell_height):

    black = np.array([0, 0, 0])

    is_black = False
    for i in range(0, image_height, cell_height):
        for j in range(0, image_width, cell_width):
        
            for x in range(i, i + cell_height):
                for y in range(j, j + cell_width):

                    # chack if any pixel in the cell is black 
                    if np.array_equal(black,image[x][y]):
                        is_black = True

           
            # if exits one: paint in black the whole cell
            if is_black:
                for x in range(i, i + cell_height):
                    for y in range(j, j + cell_width):

                        image[x][y] = black

                is_black = False
    return image

        

# in unibotics
#map_img = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')
map = read_image('mapgrannyannie.png')
dilated_image = dilate_black_pixels(map)
resized_map = resize_image(dilated_image, NEW_IMG_WIDTH, NEW_IMG_HEIGHT)
filled_map = fill_black_cells(resized_map, NEW_IMG_WIDTH, NEW_IMG_HEIGHT, CELL_WIDTH, CELL_HEIGHT)

draw_rectangles(filled_map, CELL_WIDTH, CELL_HEIGHT)

# in unibotics is showNumpy
cv2.imshow('dilated map', filled_map)
cv2.waitKey(0)
cv2.destroyAllWindows()