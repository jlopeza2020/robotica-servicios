#from GUI import GUI
#from HAL import HAL
#import cv2
import numpy as np

# Load the RGBA image and convert to NumPy array (assuming image is already loaded)
# Replace 'your_rgba_image.png' with the actual image file path

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
      matrix[i][j] = 0


while True:
    # Enter iterative code!
    GUI.showNumpy(matrix)
    print("yo")