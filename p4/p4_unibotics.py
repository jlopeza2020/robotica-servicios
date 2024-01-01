from GUI import GUI
from HAL import HAL
import cv2
import numpy as np

from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt


rgba_image = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')
kernel = np.ones((5, 5), np.uint8) 
rgba_image = cv2.erode(rgba_image, kernel, iterations=1)

height, width, channels = rgba_image.shape  # height = 279, width = 415

map_x_min, map_y_min = 0, 0
map_x_max, map_y_max = width, height

state_space = ob.RealVectorStateSpace(2)
bounds = ob.RealVectorBounds(2)
bounds.setLow(0, map_x_min)
bounds.setLow(1, map_y_min)
bounds.setHigh(0, map_x_max)
bounds.setHigh(1, map_y_max)
state_space.setBounds(bounds)

  # Iterate through the image and draw rectangles
  for i in range(0, width):
    for j in range(0, height):
      # Define the top-left and bottom-right corners of the rectangle
            
          

#print("Width:", width, "Height:", height, "Channels:", channels)
# modificaciones ppm...
# el mapa se va actualizando por si solo y no deja sacar la regresión lineal
# hacer la regesión lineal  (de 3d a 2 d y viceversa)
# obtener el path 
# moverse 



#kernel = np.ones((5, 5), np.uint8) 

#expanded_image = cv2.erode(rgba_image, kernel, iterations=1)

print(HAL.getPose3d().x,HAL.getPose3d().y)


#HAL.setV(3)
#filled_mapGUI.showNumpy(expanded_image)



while True: