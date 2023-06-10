import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og
import random
from scipy.interpolate import interpolate
import math
from math import sqrt, atan2
from collections import OrderedDict
from scipy.interpolate import splprep, splev


def wrap_angle(angle):
  return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
  """ Checks if a position or a path is valid given an occupancy map."""

  # Constructor
  def __init__(self, distance=0.22, is_unknown_valid=True):
      # map: 2D array of integers which categorizes world occupancy
      self.map = None
      # map sampling resolution (size of a cell))                          
      self.resolution = None
      # world position of cell (0, 0) in self.map                    
      self.origin = None
      # set method has been called                        
      self.there_is_map = False
      # radius arround the robot used to check occupancy of a given position               
      self.distance = distance                  
      # if True, unknown space is considered valid
      self.is_unknown_valid = is_unknown_valid  

  # Set occupancy map, its resolution and origin.
  def set(self, data, resolution, origin):
      self.map = data
      self.resolution = resolution
      self.origin = np.array(origin)
      self.there_is_map = True
      #print("is valid?", self.is_valid([0.3337898254394531, -1.2320462465286255]))

  # Given a pose, returs true if the pose is not in collision and false othewise.
  def is_valid(self, pose):
        valid = False
        th = 50 # threshold
        # Convert world robot position to map coordinates using method __position_to_map__
        cells = self.__position_to_map__(pose)
        # Check occupancy of the vicinity of a robot position (indicated by self.distance atribude).
        if cells != []:
            valid = True
            dist = int(self.distance / self.resolution)

            # robot cells margins
            min_x, max_x = cells[0]-dist, cells[0]+dist
            min_y, max_y = cells[1]-dist, cells[1]+dist

            # setting map size in x and y
            [bound_x, bound_y] = self.map.shape

            # Return True if free, False if occupied and self.is_unknown_valid if unknown.
            for i in range(min_x, max_x):
                for j in range(min_y, max_y):
                    if i < 0 or j < 0 or i >= bound_x or j >= bound_y:
                        return False # index out of bounds, invalid state
                    if self.map[i, j] > th:
                        return False # surroundings occupied, terminate once detected
                    """if self.map[i, j]==-1:
                        valid = self.is_unknown_valid # point is unknown"""
        return valid
              
  # Given a path, returns true if the path is not in collision and false othewise.
  def check_path(self, path, step_size=0.009):
        loop = len(path) - 1

        for n in range(loop):
            # getting the two points
            pose_i = np.array(path[n])
            pose_f = np.array(path[n+1])

            # finding the two points connecting line unit vector
            dx = pose_i[0] - pose_f[0]
            dy = pose_i[1] - pose_f[1]

            v = np.array([dx, dy])
            norm = np.sqrt(np.sum(v ** 2))
            vector = v / norm
            
            n = 0
            distance = math.dist(pose_i, pose_f)

            # Discretize the positions between 2 waypoints with step_size
            while n <= distance:
                cut_i = pose_i - (vector * n)
                cut_i[0] = round(cut_i[0], 4)
                cut_i[1] = round(cut_i[1], 4)

                # for each point check if `is_valid``. 
                # If only one element is not valid return False, otherwise True.
                if not self.is_valid(cut_i):
                    return False
                n += step_size
        return True
    
  # Transform position with respect to the map origin to cell coordinates
  def __position_to_map__(self, p):
       cell = np.array([0, 0])
       # convert world position to map coordinates. 
       cell[0] = int((p[0] - self.origin[0]) / self.resolution)
       cell[1] = int((p[1] - self.origin[1]) / self.resolution)
       return cell

  # Transform the cell coordinates into a world  
  def __map_to_position__(self, p): 
       # convert world position to map coordinates. 
       x = (p[0] * self.resolution) + self.origin[0]
       y = (p[1] * self.resolution) + self.origin[1]
       return [x, y]     

# ==================== CONTROLLER ====================
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_sum = 0
        self.prev_error = 0

    def compute(self, error, dt):
        # Proportional term
        p_term = self.Kp * error

        # Integral term
        self.error_sum += error * dt
        i_term = self.Ki * self.error_sum

        # Derivative term
        d_term = self.Kd * ((error - self.prev_error) / dt)
        self.prev_error = error

        # Compute PID output
        output = p_term + i_term + d_term

        return output

# Controller: Given the current position and the goal position, this function computes the desired
# linear velocity and angular velocity to be applied in order to reach the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    dx , dy = goal[0] - current[0], goal[1] - current[1]
    theta_d = math.atan2(dy, dx)
    v = 0
    th = 0.1  # threshold to check difference between angles

    # Simulation parameters
    dt = 0.1  # Time step

    # PID gains
    Kp = 1.0
    Ki = 0.5
    Kd = 0.2
    position_error = ((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2) ** 0.5

    # Create PID controllers
    pid = PIDController(Kp, Ki, Kd)

    # Compute the velocity using PID controllers
    velocity = pid.compute(position_error, dt)

    if abs(theta_d - current[2]) < th:
        v = velocity * dt
    w = Kw * wrap_angle(theta_d-current[2])
    return v, w