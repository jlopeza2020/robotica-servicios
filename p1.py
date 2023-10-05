#from GUI import GUI
#from HAL import HAL
import cv2

#import numpy as np
#import requests
#from io import BytesIO

#UNIBOTICS = 1

#from PIL import Image
# Enter sequential code!

#array = "https://github.com/JdeRobot/RoboticsAcademy/raw/master/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png"
#response = requests.get(array)

#if response.status_code == 200:
#image_stream.write(connection.read(image_len))
#image_stream.seek(0)
#  image_stream  = BytesIO(response.content)
#  file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
#  image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

#if UNIBOTICS: 
#map_img = cv2.imread('mapgrannyannie.png') 
#else: 


shape = map_img.shape
width = shape[0]
height = shape[1]


white  = np.array([255, 255, 255])
black = np.array([0, 0, 0])
#print(image)
for i in range (width):
      for j in range (height):
        if np.array_equal(white,map_img[i][j]):
        
        #print(map_img[i][j])
          #map_img[i][j] = 50

#print(image)






#while True:
    # Enter iterative code!

    #GUI.showNumpy(image)
cv2.imshow('Logo OpenCV',map_img)

cv2.waitKey(0)
cv2.destroyAllWindows()


    
    
     # Escalar y convertir a enteros en el rango [0, 255]
    #scaled_image = np.clip(image, 0, 255).astype(np.uint8)

    # Convertir el arreglo numpy en una imagen PIL
    #image_pil = Image.fromarray(scaled_image)


    #print(array)
    #shape = map_img.shape
    #width = shape[0]
    #height = shape[1]
    
    #print(width)
    
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
        
        #print(map_img[i][j])
    #      map_img[i][j] = 50