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

ORIGIN_X = 0.0
ORIGIN_Y = 0.0


def stop():
  HAL.setW(0.0)
  HAL.setV(0.0)
  
def navigate(current_3d_x, current_3d_y, objective_3d_x, objective_3d_y, current_angle):
    
    has_reached = False 
    
    # go to each point
    diff_x = objective_3d_x - current_3d_x
    diff_y = objective_3d_y - current_3d_y
    
    objective_angle = math.atan2(diff_y, diff_x)
    
    angle_diff = current_angle - objective_angle
      
    # Navigation logic 
    if (angle_diff > math.pi):
      angle_diff -= 2*math.pi
    elif(angle_diff < -math.pi):
      angle_diff +=2*math.pi
    
    # means that it is so close that it goes forward 
    if abs(angle_diff) < 0.4:
      HAL.setV(0.1)
      HAL.setW(0.0)
    else:
      
      HAL.setV(0.01)
      if(angle_diff > 0.0):
      
        HAL.setW(-0.10)
      else: 
        HAL.setW(0.10)

    # has reached to the objective
    if (abs(diff_x) <= 0.1 and abs(diff_y) <= 0.1):
      has_reached = True
    
    return has_reached
  
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
      
            if image[i][j][0] < 0.7 and image[i][j][1] < 0.7  and image[i][j][2] < 0.7:
                # x, y ,radius 
                obstacles.append([i, j, 2])
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


def isStateValid(state, robot_width, robot_length):
    x = state.getX()
    y = state.getY()

    # Check if the state is inside any obstacle or collides with the robot
    for obstacle in obstacles:
        obstacle_x, obstacle_y, obstacle_radius = obstacle
        if (
            sqrt(pow(x - obstacle_x, 2) + pow(y - obstacle_y, 2)) - obstacle_radius <= 0
            or abs(x - robot_width / 2) < 0 or abs(x - dimensions[2] + robot_width / 2) < 0
            or abs(y - robot_length / 2) < 0 or abs(y - dimensions[3] + robot_length / 2) < 0
        ):
            return False
    return True


def plan(origin_x, origin_y, destination_x, destination_y, r_w, r_l):
    # Construct the robot state space in which we're planning.
    space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    # Set state space's lower and upper bounds
    bounds.setLow(0, dimensions[0] + r_w / 2)  # Adjust for robot width
    bounds.setLow(1, dimensions[1] + r_l / 2)  # Adjust for robot length
    bounds.setHigh(0, dimensions[2] - r_w / 2)  # Adjust for robot width
    bounds.setHigh(1, dimensions[3] - r_l / 2)  # Adjust for robot length

    space.setBounds(bounds)
    
    
    
    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    # Set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(lambda state: isStateValid(state, r_w, r_l)))

    y_init, x_init = rw2map(origin_x,origin_y) 
    y_objective, x_objective = rw2map(destination_x,destination_y)
    
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

kernel = np.ones((10, 10), np.uint8) 
# Thick obstacles
rgb_image = cv2.erode(rgb_image, kernel, iterations=1)

# Set dimensions in pixels
dimensions = [0, 0, 279, 415] 

# Extract obstacles from the image
obstacles = extract_obstacles(rgb_image)


# create init-end point path 
# set robot dimensions: is a point
robot_width = 0  # metres
robot_length = 0  # metres

solution_path = None
while solution_path is None: 
  
    solution_path = plan(0,0, SHX, SH1Y, robot_width, robot_length)

    if solution_path is not None:
        inverted_solution = invert_array(solution_path)
        complete_path_map = add_intermediate_points(inverted_solution)
    else:
        print("No valid solution found")

# structure [x_rw, y_rw] 
complete_path_rw = convert_path_to_rw(complete_path_map) 


# create end-init point path 

# set robot dimensions 
robot_width_2 = 2.5  # meters
robot_length_2 = 4.0  # meters

solution_path_2 = None
while solution_path_2 is None: 
  
    solution_path_2 = plan(SHX, SH1Y, 0, 0, robot_width_2, robot_length_2)

    if solution_path_2 is not None:
        inverted_solution_2 = invert_array(solution_path_2)
        complete_path_map_2 = add_intermediate_points(inverted_solution_2)
      
    else:
        print("No valid  back solution found")

# structure [x_rw, y_rw] 
complete_path_rw_2 = convert_path_to_rw(complete_path_map_2) 

pos_move_coords = 0
phase_go_sh = True
phase_lift = False 
phase_go_back = False
phase_put_down = False 
while True:
  
    cu_3d_x = HAL.getPose3d().x 
    cu_3d_y = HAL.getPose3d().y
    cu_angle = HAL.getPose3d().yaw
    
    if(phase_go_sh): 
      
      
      GUI.showPath(complete_path_map)

      ob_3d_x =  complete_path_rw[pos_move_coords][0]
      ob_3d_y =  complete_path_rw[pos_move_coords][1]

      # navigate from current point to objective 
      has_reached = navigate(cu_3d_x, cu_3d_y, ob_3d_x, ob_3d_y, cu_angle)
      
      if(has_reached): 
      
        if(pos_move_coords < (len(complete_path_rw)-1)):
        
          pos_move_coords = pos_move_coords + 1
          has_reached = False

        else:
          stop()
          phase_go_sh = False
          phase_lift = True
          pos_move_coords = 0 
          
    if(phase_lift): 
      HAL.lift()
      phase_lift = False
      phase_go_back = True
      
      
    if(phase_go_back): 
      
      GUI.showPath(complete_path_map_2)

      ob_3d_x =  complete_path_rw_2[pos_move_coords][0]
      ob_3d_y =  complete_path_rw_2[pos_move_coords][1]

      # navigate from current point to objective 
      has_reached = navigate(cu_3d_x, cu_3d_y, ob_3d_x, ob_3d_y, cu_angle)
      
      if(has_reached): 
      
        if(pos_move_coords < (len(complete_path_rw)-1)):
        
          pos_move_coords = pos_move_coords + 1
          has_reached = False

        else:
          stop()
          phase_go_back = False
          phase_put_down = True
      
    if(phase_put_down): 
      HAL.putdown()
      phase_put_down = False