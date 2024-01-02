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


def invert_array(array):
  
    length = len(array)
    inverted_array = np.zeros((length, 2))
    #array = np.zeros((length, 2))
    for i in range(length):
        inverted_array[i][0] = array[i][1]
        inverted_array[i][1] = array[i][0]
    
    return inverted_array   

def extract_obstacles(image): 
    obstacles = []
    height, width, _ = image.shape  # Get image dimensions
    for i in range(0, height-1):
        for j in range(0, width-1):
            if image[i][j][0] < 0.5 and image[i][j][1] < 0.5 and image[i][j][2] < 0.5:
                obstacles.append([i, j])
    return obstacles

# from meter to pixel 
def rw2map(x_rw, y_rw):
    x_map = 139.5 - 20.515 * x_rw
    y_map = 207.5 - 20.16 * y_rw
    return (round(y_map), round(x_map))

def isStateValid(state):
    x = state.getX()
    y = state.getY()

    # Check if the state is inside any obstacle
    for obstacle in obstacles:
        obstacle_x, obstacle_y = rw2map(*obstacle)
        distance = sqrt(pow(x - obstacle_x, 2) + pow(y - obstacle_y, 2))

        if distance < 0.5:  # Adjust this threshold as needed
            return False
    return True

def plan():
    # Construct the robot state space in which we're planning.
    space = ob.SE2StateSpace()

    # Set state space's lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])
    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])
    space.setBounds(bounds)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    # Set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    y_init, x_init = rw2map(0,0) 
    y_objective, x_objective = rw2map(3.728,0.579)
    
    # Set our robot's starting and goal state
    start = ob.State(space)
    start().setX(x_init)
    start().setY(y_init)
    start().setYaw(math.pi / 4)
    goal = ob.State(space)
    goal().setX(x_objective)
    goal().setY(y_objective)
    goal().setYaw(math.pi / 4)

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create a planner for the defined space
    planner = og.RRTConnect(si)

    # Set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)

    # Perform setup steps for the planner
    planner.setup()

    # Solve the problem and print the solution if it exists
    solved = planner.solve(1.0)
    if solved:
        solution_path = pdef.getSolutionPath()
        matrix = solution_path.printAsMatrix()
        path = create_numpy_path(matrix)
        print(path)
        return path
    else: 
        print("NO SOLUTION FOUND")
        
    #return path 

def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

# GET IMAGE
rgb_image = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')

kernel = np.ones((5, 5), np.uint8) 
# Thick obstacles
rgb_image = cv2.erode(rgb_image, kernel, iterations=1)

# Image dimensions
height, width, channels = rgb_image.shape

# Set dimensions in pixels
dimensions = [0, 0, 279, 415] 
# Extract obstacles from the image
obstacles = extract_obstacles(rgb_image)
# Plan the path
solution_path = plan()


# iterate over the image and set pixels like [0.0,0.0,0.0] (black)  as obstacles 
#for i in range(0, height-1):
  #for j in range(0, width-1):
    
  #  if(rgb_image[i][j][0] < 0.5 and rgb_image[i][j][1] < 0.5 and rgb_image[i][j][2] < 0.5):
    
  #    obstacles.append([i,j])
      
# SET DIMENSION IN PIXELS
#dimensions = [0, 0, 279, 415] 

      
#print(counter)
#print(obstacles)      # Define the top-left and bottom-right corners of the 
      #guardas las coordenadas 
      

#map_x_min, map_y_min = 0, 0
#map_x_max, map_y_max = width, height

#state_space = ob.RealVectorStateSpace(2)
#bounds = ob.RealVectorBounds(2)
#bounds.setLow(0, map_x_min)
#bounds.setLow(1, map_y_min)
#bounds.setHigh(0, map_x_max)
#bounds.setHigh(1, map_y_max)
#state_space.setBounds(bounds)


# para x_map e y_map hay que hacer round
# expected path to obtain
# init point: [206,140] -> [0,0]
# intermediate point : [196,90]
# goal point: [197,62] -> [3.728, 0.579]

  # Iterate through the image and draw rectangles
 # for i in range(0, width):
#    for j in range(0, height):
      # Define the top-left and bottom-right corners of the 
      #guardas las coordenadas 
      
# trabaja en metros y lo pasa a pixeles para el path          
          
# obtener el path 
# moverse 

# real world dimensions 
#dimensions = [0, 0, 20.62, 13.6] 
#obstacle = [5.5, 5.5, 1]   # [x, y, radius] # poner los obstáculos 


#print(HAL.getPose3d().x,HAL.getPose3d().y)

# init point
y_init, x_init = rw2map(0,0) 
y_objective, x_objective = rw2map(3.728,0.579)

#print(y_init, x_init)   
#print(y_objective, x_objective)

#array = [[206,140], [196,90], [197,62]]
#array = [[y_init, x_init], [y_objective, x_objective]]

# = np.transpose(solution_path).tolist()

#print("soy invertido  " + str(inverted_array))

#aux_Array = []

#print(len(solution_path))
"""
length = len(solution_path)
inverted_array = np.zeros((length, 2))
array = np.zeros((length, 2))
for i in range(length):
        inverted_array[i][0] = solution_path[i][1]
        inverted_array[i][1] = solution_path[i][0]
    
print(inverted_array)   
"""
inverted_solution = invert_array(solution_path)
print(inverted_solution)

GUI.showPath(inverted_solution)

# 
while True:
    #HAL.setV(3)
    #print(HAL.getPose3d().x,HAL.getPose3d().y)