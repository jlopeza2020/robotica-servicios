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
                obstacles.append([i, j])
    return obstacles

# from meter to pixel 
def rw2map(x_rw, y_rw):
    x_map = 139.5 - 20.515 * x_rw
    y_map = 207.5 - 20.16 * y_rw
    return (round(y_map), round(x_map))

def isStateValid(state):
  
    #print(state)
    x = state.getX()
    #print(x)
    y = state.getY()
    #print(y)

    # Check if the state is inside any obstacle
    for obstacle in obstacles:
        #print(rw2map(*obstacle))
        obstacle_x, obstacle_y = rw2map(*obstacle)
        distance = sqrt(pow(x - obstacle_x, 2) + pow(y - obstacle_y, 2))

        if distance < 0.8:  
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
    # tiene muchos que se quedan en los obstáculos 
    #planner = og.RRTConnect(si)
    # solo da el punto de entrada y salida
    #planner = og.PRM(si)
    # no todos los puntos llegan al objetivo
    ##planner = og.EST(si)
    # MALLL 
    #planner = og.KPIECE1(si)
    # NO CAMBIA SOLO PRINCIPIO Y FIN
    #planner = og.LazyPRM(si)
    # MUCHOS PUNTOS Y CAMINO MUY ENREVESADO
    #planner = og.SBL(si)
    # NO VALID SOLUTION FOUND 
    #planner = og.FMT(si)
    # PUNTO DE INICIO Y FINAL
    #planner = og.ABITstar(si)
    #planner = og.PRMstar(si)
    # NO LLEGA AL OBJETIVO
    #planner = og.InformedRRTstar(si)
    







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
        #print(path)
        return path
    else: 
        print("NO SOLUTION FOUND")
        

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


#kernel = np.ones((5, 5), np.uint8) 
# Thick obstacles
#rgb_image = cv2.erode(rgb_image, kernel, iterations=1)

# Set dimensions in pixels
dimensions = [0, 0, 279, 415] 
# Extract obstacles from the image
obstacles = extract_obstacles(rgb_image)
# Plan the path
"""
solution_path = plan()


# expected path to obtain
# init point: [206,140] -> [0,0]
# intermediate point : [196,90]
# goal point: [197,62] -> [3.728, 0.579]

#print(HAL.getPose3d().x,HAL.getPose3d().y)

y_init, x_init = rw2map(0,0) 
y_objective, x_objective = rw2map(3.728,0.579)

inverted_solution = invert_array(solution_path)
print(inverted_solution)

GUI.showPath(inverted_solution)
"""

while True:
    #HAL.setV(3)
    #print(HAL.getPose3d().x,HAL.getPose3d().y)
    
    #obstacles = extract_obstacles(rgb_image)
    # Plan the path
    solution_path = plan()


    # expected path to obtain
    # init point: [206,140] -> [0,0]
    # intermediate point : [196,90]
    # goal point: [197,62] -> [3.728, 0.579]

    #print(HAL.getPose3d().x,HAL.getPose3d().y)

    #y_init, x_init = rw2map(0,0) 
    #y_objective, x_objective = rw2map(3.728,0.579)
    if solution_path is not None:
        inverted_solution = invert_array(solution_path)
        print(inverted_solution)
        GUI.showPath(inverted_solution)
    else:
      print("No valid solution found.")
    
    #inverted_solution = invert_array(solution_path)
    #print(inverted_solution)
    
    #GUI.showPath(inverted_solution)