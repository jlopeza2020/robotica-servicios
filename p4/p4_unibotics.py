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


def extract_obstacles(image): 
  
  obstacles = []
  
  for i in range(0, height-1):
    for j in range(0, width-1):
    
      if(image[i][j][0] < 0.5 and image[i][j][1] < 0.5 and image[i][j][2] < 0.5):
    
        obstacles.append([i,j])
        
  return obstacles
  
# from meter to pixel 
def rw2map(x_rw, y_rw):
  x_map = 139.5 - 20.515*x_rw
  y_map = 207.5 -20.16*y_rw
  
  return (round(y_map),round(x_map))


def isStateValid(state):
  
  x = state.getX()
  y = state.getY()
  
  # if distance actual to objective is slower than 0.5 is invalid state 
  if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0.5:
    return False
  return True
  
def plan():
  # Construct the robot state space in which we're planning. We're
  # planning in [0,1]x[0,1], a subset of R^2.
  space = ob.SE2StateSpace()

  # set state space's lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(0, dimensions[0])
  bounds.setLow(1, dimensions[1])
  bounds.setHigh(0, dimensions[2])
  bounds.setHigh(1, dimensions[3])
  space.setBounds(bounds)

  # construct a space information instance for this state space
  si = ob.SpaceInformation(space)
  # set state validity checking for this space
  si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

  # Set our robot's starting and goal state
  start = ob.State(space)
  start().setX(0)
  start().setY(0)
  start().setYaw(math.pi / 4)
  goal = ob.State(space)
  goal().setX(10)
  goal().setY(10)
  goal().setYaw(math.pi / 4)

  # create a problem instance
  pdef = ob.ProblemDefinition(si)

  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(si)

  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(1.0)
  if solved:
    print(pdef.getSolutionPath())
    #plot_path(pdef.getSolutionPath(), dimensions)
    matrix = solution_path.printAsMatrix()
    path = create_numpy_path(matrix)

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
# thick obstacles
rgb_image = cv2.erode(rgb_image, kernel, iterations=1)

# height map = 279, width  mao = 415
height, width, channels = rgb_image.shape


dimensions = [0, 0, 279, 415] 
obstacles = extract_obstacles(rgb_image)
plan()

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

print(y_init, x_init)   
print(y_objective, x_objective)

#array = [[206,140], [196,90], [197,62]]
array = [[y_init, x_init], [y_objective, x_objective]]
GUI.showPath(array)

# 
while True:
    #HAL.setV(3)
    #print(HAL.getPose3d().x,HAL.getPose3d().y)