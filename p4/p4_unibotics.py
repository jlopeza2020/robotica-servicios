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

SHX = 3.728
SH1Y = 0.579
SH2Y = -1.242
SH3Y = -3.039
SH4Y = -4.827 
SH5Y = -6.781
SH6Y = -8.665

# Function to convert all coordinates from map to real-world
def convert_path_to_rw(path_map):
    path_rw = []
    for point_map in path_map:
        y_map, x_map = point_map
        y_rw, x_rw = map2rw(x_map, y_map)
        path_rw.append([x_rw, y_rw])
    return np.array(path_rw)
    
def add_intermediate_points(path):
    # Add intermediate points to the path
    num_points = len(path)
    intermediate_points = []

    for i in range(num_points - 1):
        # Add the original point
        intermediate_points.append(path[i])

        # Calculate and add intermediate points
        delta_x = (path[i + 1][0] - path[i][0]) / 10.0  
        delta_y = (path[i + 1][1] - path[i][1]) / 10.0

        for j in range(1, 10):  # Add 9 intermediate points between each pair of original points
            intermediate_x = path[i][0] + j * delta_x
            intermediate_y = path[i][1] + j * delta_y
            intermediate_points.append([intermediate_x, intermediate_y])

    # Add the last point in the original path
    intermediate_points.append(path[-1])

    return np.array(intermediate_points)
    
def invert_array(array):
  
    length = len(array)
    inverted_array = np.zeros((length, 2))

    for i in range(length):
        inverted_array[i][0] = array[i][1]
        inverted_array[i][1] = array[i][0]
    
    return inverted_array   

def extract_obstacles(image): 
    obstacles = []
    height, width, _ = image.shape  # Get image dimensions
    for i in range(height):
        for j in range(width):
      
            if image[i][j][0] < 0.5 and image[i][j][1] < 0.5  and image[i][j][2] < 0.5:
                # x, y ,radius 
                obstacles.append([i, j, 1])
    return obstacles

# from meter to pixel 
def rw2map(x_rw, y_rw):
    x_map = 139.5 - 20.515 * x_rw
    y_map = 207.5 - 20.16 * y_rw
    return (round(y_map), round(x_map))
 
def map2rw(x_map, y_map):
    x_rw =  6.8 -0.049* x_map
    y_rw =  10.310 -0.05* y_map
    return (y_rw, x_rw)

def isStateValid(state):
  
    x = state.getX()
    y = state.getY()

    # Check if the state is inside any obstacle
    for obstacle in obstacles:
        
        if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0:
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
    y_objective, x_objective = rw2map(SHX,SH1Y)
    
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
    #planner = og.EST(si)
    planner = og.PRM(si)
    #planner = og.RRTConnect(si)

    # Set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)

    # Perform setup steps for the planner
    planner.setup()
    
    # Solve the problem and print the solution if it exists
    solved = planner.solve(1.0)
    if solved:
      solution_path = pdef.getSolutionPath()
      if solution_path:
          matrix = solution_path.printAsMatrix()
          path = create_numpy_path(matrix)
          return path
      else:
          print("NO SOLUTION FOUND: getSolutionPath returned None")
    else: 
      print("NO SOLUTION FOUND: Planner failed to solve")
    

def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

# get image
rgb_image = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')

kernel = np.ones((6, 6), np.uint8) 
# Thick obstacles
rgb_image = cv2.erode(rgb_image, kernel, iterations=1)

# Set dimensions in pixels
dimensions = [0, 0, 279, 415] 
# Extract obstacles from the image
obstacles = extract_obstacles(rgb_image)

"""
solution_path = None
while solution_path is None: 
  solution_path = plan()

  if solution_path is not None:
    inverted_solution = invert_array(solution_path)
    print(inverted_solution)
    GUI.showPath(inverted_solution)
  else:
    print("No valid solution found.")
"""
solution_path = None
while solution_path is None: 
    solution_path = plan()

    if solution_path is not None:
        inverted_solution = invert_array(solution_path)
        print(inverted_solution)
        complete_path_map = add_intermediate_points(inverted_solution)
        #print("intermediate"+ str(intermediate_solution))
        print(complete_path_map)
        GUI.showPath(complete_path_map)
    else:
        print("No valid solution found.")

# structure [x_rw, y_rw] 
complete_path_rw = convert_path_to_rw(complete_path_map) 
print(complete_path_rw)

Kp = 1.5
pospos_move_coords = 1
phase_go_sh = True
while True:
  
    current_3d_x = HAL.getPose3d().x 
    current_3d_y = HAL.getPose3d().y
    current_angle = HAL.getPose3d().yaw
    
    if(phase_go_sh): 
      
      #if(goal_array_pos < len(complete_path_rw)):
        
        # go to each point
        

      diff_x = complete_path_rw[pos_move_coords][0] - current_3d_x
      diff_y = complete_path_rw[pos_move_coords][1] - current_3d_y
    
      objective_angle = math.atan2(diff_y, diff_x)
    
      angle_diff = current_angle - objective_angle
    
        # Navigation logic 
      if (angle_diff > math.pi):
        angle_diff -= 2*math.pi
      elif(angle_diff < -math.pi):
        angle_diff +=2*math.pi
     
      if(angle_diff > 0):
        HAL.setW(0.5 + Kp*angle_diff)
      else: 
        HAL.setW(-0.5 - Kp*angle_diff)
      
        # means that it is so close that it goes forward 
      if abs(angle_diff) < 0.1:
        HAL.setV(1.0)
      else:
        HAL.setV(0.0)
      
      if (abs(diff_x) <= 0.1 and abs(diff_y) <= 0.1):
        has_reached = True
   
      if(has_reached): 
      
        if(pos_move_coords < len(complete_path_rw)):
        
          pos_move_coords = pos_move_coords + 1
          has_reached = False
        else:
          phase_go_sh = False
      
      
      
    #if
    """
    solution_path = plan()

    if solution_path is not None:
        inverted_solution = invert_array(solution_path)
        print(inverted_solution)
        GUI.showPath(inverted_solution)
    else:
      print("No valid solution found.")
    """
    ## vaya al objetivo 
    ## coja la estantería 
    ## vaya la destino (ya cambia su dimensión) 
    ##  deje la estantería 
    
    