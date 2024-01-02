import cv2
import numpy as np

# Load the image from the file
image = cv2.imread('/home/juloau/Desktop/CUARTO/PRIMER_CUATRI/ROBOTICA_SERVICIOS/REPO/robotica-servicios/p4/map.png')

# Check if the image was loaded successfully
if image is not None:
    # Get image dimensions
    height, width, _ = image.shape

    # Iterate over each pixel in the image
    #for i in range(height):
    #    for j in range(width):
            #print(image[i, j])


    # Define el kernel o elemento estructurante para la dilatación
    kernel = np.ones((10,10), np.uint8)  # Puedes ajustar el tamaño del kernel según tus necesidades

    # Aplica la dilatación a la imagen
    cv2.erode(image, kernel, iterations=1)

    # Optionally, display the image
    cv2.imshow('Image', image)
    #cv2.imshow('DELATTED IMAGE', dilated_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    print("Failed to load the image.")
