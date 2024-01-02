import cv2

# Load the image from the file
image = cv2.imread('/home/juloau/Desktop/CUARTO/PRIMER_CUATRI/ROBOTICA_SERVICIOS/REPO/robotica-servicios/p4/map.png')

# Check if the image was loaded successfully
if image is not None:
    # Get image dimensions
    height, width, _ = image.shape

    # Iterate over each pixel in the image
    for i in range(height):
        for j in range(width):
            print(image[i, j])

    # Optionally, display the image
    cv2.imshow('Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    print("Failed to load the image.")
