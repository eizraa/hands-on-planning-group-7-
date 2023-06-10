#!/usr/bin/python3
import numpy as np
import rospy
import tf
import random
import time
from math import sqrt, atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from utils_lib.online_planning import StateValidityChecker, move_to_point
from RRT import *
import math
from scipy.interpolate import interpolate
import math
from collections import OrderedDict
from scipy.interpolate import splprep, splev

class OnlinePlanner:
  # OnlinePlanner Constructor
  def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, distance_threshold):

      # ==================== ATTRIBUTES ====================
      # List of turple points which define the plan. None if there is no plan
      self.path = []
      # State Validity Checker object                                               
      self.svc = StateValidityChecker(distance_threshold)
      # Current robot SE2 pose [x, y, yaw], None if unknown          
      self.current_pose = (2.149, -0.7, 0.7061)
      # Goal where the robot has to move, None if it is not set                                                                 
      self.goal = None
      # Last time a map was received (to avoid map update too often)                                              
      self.last_map_time = rospy.Time.now()
      # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                         
      self.dominion = dominion              
      # Random generated points list
      self.random_points = []  
      self.valid_pts = []
      # Best goals points and its list
      self.best_target_point = None
      self.goals_list = []   
      # map environment
      self.env = []                                
      # map width and height
      self.width = None
      self.height = None     
      # Status of goal reached
      self.goal_reached = False  
      # Map origin and resolution
      self.map_origin = None
      self.map_resolu = None
      # Path planning counter
      self.counter = 0

      # ==================== CONTROLLER PARAMETERS ====================
      # Proportional linear velocity controller gain
      self.Kv = 0.5
      # Proportional angular velocity controller gain                 
      self.Kw = 0.5
      # Maximum linear velocity control action                 
      self.v_max = 0.5
      # Maximum angular velocity control action             
      self.w_max = 0.5              

      # ==================== PUBLISHERS ====================
      # Publisher for sending velocity commands to the robot
      self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
      # Publisher for visualizing the path to point with rviz
      self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
      # Publisher for visualizing the random points with rviz
      self.points_pub = rospy.Publisher('~points_marker', Marker, queue_size=1)
      # Publisher for visualizing the best random point with rviz
      self.best_pub = rospy.Publisher('~goal_marker', Marker, queue_size=1)

      # ==================== SUBSCRIBERS ====================
      # subscriber to gridmap_topic from Octomap Server
      self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap) 
      # subscriber to odom_topic
      self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom) 
      
      # ==================== TIMER ====================
      # Timer for velocity controller
      rospy.Timer(rospy.Duration(0.1), self.controller)

   # compute the euclidean distance between two points
  def euclidean_distance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance

  # ==================== CALL-BACK FUNCTIONS ====================
  # Map callback:  Gets the latest occupancy map published by Octomap server and update
  # the state validity checker
  def get_gridmap(self, gridmap):
      #if self.counter<5:
        # to avoid map update too often (change value if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:          
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            self.env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            #print("env: \n", self.env)
            self.map_origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.map_resolu = gridmap.info.resolution
            self.svc.set(self.env, self.map_resolu, self.map_origin)

            # map width and height
            # print("map size is {}".format(env.shape))
            self.height, self.width = self.env.shape   

            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path

                # check total_path validity. 
                valid = self.svc.check_path(total_path)
                # If total_path is not valid make self.path = None and replan
                #if not valid and self.counter<4:
                #print("counter", self.counter)
                """if not valid and self.counter<5:
                    print("\nOBSTACLE FOUND")  
                    self.path = []
                    self.__send_commnd__(0, 0)
                    print("REGENERATE PATH")
                    self.path = self.plan()
                    self.counter +=1
                elif self.counter>=5:
                    #self.goal = None
                    #self.best_target_point = None
                    print("OBBSTTACCLLLEEEE XXXXXXX")
                    self.path = []
                    self.__send_commnd__(0, 0)
                    self.goal_reached = True
                    self.counter = 0
                    print("Max regenerating time, select another goal.") """                      

  # Odometry callback: Gets current robot pose and stores it into self.current_pose
  def get_odom(self, odom):
      _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
      self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
 
  # ==================== SAMPLE-BASED ALGORITHM FUNCTIONS ====================  
  def generate_random_point(self):
        # Generate a random point within the map boundaries
        # setting map size in x and y
        #[self.width, self.height] = self.svc.map.shape
        print("\n\nGenerating random points.")
        self.valid_pts = []
        self.goal_reached = False
        points = []
        while len(points)<200:
            #x = random.uniform(0, 2.5)
            #y = random.uniform(-0.6, 1.6)
            #x = random.uniform(0, 5.6)
            #y = random.uniform(-1, 3.5)
            #x = random.uniform(-0.778, 6.7)
            #y = random.uniform(-2.27, 4.39)
            x = random.uniform(self.dominion[0], self.dominion[1])
            y = random.uniform(self.dominion[0], self.dominion[1])
            point = (x, y)
            map_x, map_y=self.svc.__position_to_map__(point)
            if self.svc.is_valid(point) and self.env[map_x, map_y]==0:
                points.append(point)
        return points
  
  def count_unexplored_cells(self, target_point, radius):
        # Count the number of unexplored cells around the target_point within the specified radius
        unexplored_count = 0
        map_x, map_y = self.svc.__position_to_map__(target_point)
        #map_x, map_y = self.svc.__position_to_map__(target_point, self.map_origin, self.map_resolu)
        # print("target point is {} in world position".format(target_point))
        # print("target point is {} and {} in map position".format(map_x, map_y))

        min_i, max_i = max(0, map_x - radius), min(self.height, map_x + radius)
        min_j, max_j = max(0, map_y - radius), min(self.width, map_y + radius)

        for i in range(min_i, max_i):
            for j in range(min_j, max_j):
                # print("coordinate:({},{})".format(i,j))
                # print("value = {}".format(self.env[i,j]))
                if self.env[i,j] == -1:
                    unexplored_count += 1
                    
                elif self.env[i,j] == 100:
                    unexplored_count -= 1

        #print("unexplored={}".format(unexplored_count))
        return unexplored_count
    
  def information_gain(self, robot_position, target_point, radius=15):
        # Calculate the information gain based on distance to target_point and number of unexplored cells
        distance_gain = np.sqrt((target_point[0]-robot_position[0])**2 + (target_point[1]-robot_position[1])**2)
        unexplored_gain = self.count_unexplored_cells(target_point, radius)
        total_gain = -3/distance_gain + unexplored_gain/20
        #total_gain = 1/distance_gain + unexplored_gain/10
        return total_gain
  
  # Set a new goal and compute a plan to it using self.plan() method
  def set_goal(self):
      self.valid_pts = []
      self.best_target_point = None
      # Perform exploration by selecting a random point with the highest information gain
      if self.svc.there_is_map:
        self.random_points = self.generate_random_point()
        #print(self.random_points)

        max_information_gain = -1000
        best_target_point = None

        for point in self.random_points:
            #if self.svc.is_valid(point, self.map_origin, self.map_resolu, self.env):
            info_gain = self.information_gain(self.current_pose, point)
            #print("{} has {} gain".format(point, info_gain))
            #print("vaild point")
            dist_counter=0
            distance = self.euclidean_distance(self.current_pose[0], self.current_pose[1], point[0], point[1])

            #if info_gain > max_information_gain and distance>0.5 and distance<2:
            if info_gain > max_information_gain and distance>0.5 and distance<3:
                max_information_gain = info_gain
                best_target_point = point
            #else:
                #print("{} is invalid or still unknown".format(point))


        self.publish_points()
        self.best_target_point = best_target_point
        self.publish_goal()

        print("Best point is {} ".format(best_target_point))
        print("Max gain is {}".format(max_information_gain))
        return self.best_target_point

  # ==================== PATH-PLANNING FUNCTIONS ====================
  def explore(self):
        while True:
            self.counter=0
            best_target_point = self.set_goal()

            while best_target_point is None:
                #print("All points are invalid, regenerate.")
                best_target_point = self.set_goal()

            if best_target_point is not None:
                self.goals_list.append(best_target_point)
                print("New goal received: ({}, {})".format(best_target_point[0], best_target_point[1]))
                self.goal = best_target_point

                # to send zero velocity while planning
                self.path = []                                                 
                self.path = self.plan()

            """if self.path is None or len(self.path)==0:
                self.goal_reached=True"""
            
            #time.sleep(10)
            
            while self.goal_reached is False:
                   time.sleep(1)

  # Solve plan from current position to self.goal.
  def plan(self):
      # List of waypoints [x, y] that compose the plan
      path = []
      trial = 0
      # If planning fails, allow replanning for several trials
      while len(path) == 0 and trial < 5:
          print("Compute new path")
          start = self.svc.__position_to_map__(self.current_pose[:2])
          goal = self.svc.__position_to_map__(self.goal)
         
          # Plan a path from self.current_pose to self.goal
          grid_map = self.env
          """grid_map[grid_map > 50] = 1
          grid_map[grid_map <= 50] = 0"""
          path_RRT = RRTT(grid_map, 10000, 3, 0.2, start, goal, self.map_resolu)
          new_path = []
          if path_RRT is not None:
            for i in path_RRT:
                new_path.append(self.svc.__map_to_position__(i))

          path=new_path
          distance = self.euclidean_distance(self.current_pose[0], self.current_pose[1], self.goal[0], self.goal[1])
          if len(path)>2:
            if distance>3:
                points_number=25
            else:
                points_number=15
            path = self.smooth_path_bspline(path, points_number)

          # check total_path validity. 
          valid = self.svc.check_path(path)
          # If total_path is not valid make self.path = None and replan
          if not valid:
            print("Obstacle found, replan.")
            path = []
            #print("bspline path:", path)
          #print("new path", path)
          trial += 1
          print("\n\n\n\n")
          
      if trial == 5:
          # If planning fails, consider increasing the planning time
          print("Path not found!")
          self.goal_reached = True
          path = []
          #self.__send_commnd__(-0.3, 0)
          #time.sleep(1)
          return path
      else:
          print("Path found")
          # Publish plan marker to visualize in rviz
          self.publish_path(path)
          # remove initial waypoint in the path (current pose is already reached)
          del path[0]               
          return path
     
  # This method is called every 0.1s. It computes the velocity comands in order to reach the
  # next waypoint in the path. It also sends zero velocity commands if there is no active path.
  def controller(self, event):
      v = 0
      w = 0
      if self.path is not None and len(self.path) > 0:

          # If current wait point reached with some tolerance move to next way point, otherwise move to current point
          #if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < 2*self.svc.resolution:
          if np.linalg.norm(np.array(self.path[0]) - np.array(self.current_pose[0:2])) < 2 * self.svc.resolution:
              #print("Position {} reached".format(self.path[0]))
              del self.path[0]
              if len(self.path) == 0:
                  self.goal = None
                  self.goal_reached = True
                  self.counter = 0
                  print("Final position reached!")

                  # check if exploring should be stopped
                  count = 0
                  for i in range(0, self.height):
                    for j in range(0, self.width):
                        if self.env[i,j] == -1:
                            count += 1
                  
                  # ratio of unexplored cells in whole map
                  ratio = count / (self.width * self.height)
                  print("Exploartion ratio:", ratio)
                  if ratio <= 0.05:
                      self.stop_exploration()

          else:
              # Compute velocities using controller function in utils_lib
              v, w = move_to_point(self.current_pose, self.path[0])
              if v > self.v_max:
                  v = self.v_max
              if w > self.w_max:
                  w = self.w_max
    
      # Publish velocity commands
      self.__send_commnd__(v, w)

  def stop_exploration(self):
      print("Exploration is done.")
      self.__send_commnd__(0, 0)
      rospy.signal_shutdown("Received stop signal")

  # ==================== B-SPLINE FUNCTIONS ====================
  # generates equidistant points on a line segment between two points 
  def points_on_line(self, x1, y1, x2, y2, n):
    dx = (x2-x1) / (n-1)
    dy = (y2-y1) / (n-1)

    # returns a list of evenly distributed coordinates along the line segment
    return [(x1 + i*dx, y1 + i*dy) for i in range(n-1)] + [(x2, y2)]

  # smooths a path using B-spline interpolation
  def smooth_path_bspline(self, points, p_num):
    bspline_points = []
    x = []
    y = []
    for i in range(0, len(points) - 1):
        points_num = int(self.euclidean_distance(points[i][0], points[i][1], points[i+1][0], points[i+1][1]))
        points_list = self.points_on_line(points[i][0], points[i][1], points[i+1][0], points[i+1][1], (points_num)+2)
        for p in points_list:
            bspline_points.append(p)

    new_dict = OrderedDict.fromkeys(bspline_points)
    new_list = list(new_dict.keys())

    # create new x and y with removing duplicated points
    for point in new_list:
        x.append(point[0])
        y.append(point[1])
    
    tck, *rest = splprep([x, y], s=0.0000001**5)
    u = np.linspace(0, 1, num=p_num)
    smooth= splev(u, tck)

    # create new smoothed, curved path list 
    path = []
    for i in range(0, len(smooth[0])):
        path.append((smooth[0][i], smooth[1][i]))
    return path

  # ==================== PUBLISHER HELPERS ====================
  # Transform linear and angular velocity (v, w) into a Twist message and publish it
  def __send_commnd__(self, v, w):
      cmd = Twist()
      cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
      cmd.linear.y = 0
      cmd.linear.z = 0
      cmd.angular.x = 0
      cmd.angular.y = 0
      cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
      self.cmd_pub.publish(cmd)

  # Publish a path as a series of line markers in red
  def publish_path(self, path):
      if len(path) > 1:
          print("Publish path!")
          m = Marker()
          m.header.frame_id = 'world_ned'
          m.header.stamp = rospy.Time.now()
          m.id = 0
          m.type = Marker.LINE_STRIP
          m.ns = 'path'
          m.action = Marker.DELETE
          m.lifetime = rospy.Duration(0)
          self.marker_pub.publish(m)

          m.action = Marker.ADD
          m.scale.x = 0.1
          m.scale.y = 0.0
          m.scale.z = 0.0
        
          m.pose.orientation.x = 0
          m.pose.orientation.y = 0
          m.pose.orientation.z = 0
          m.pose.orientation.w = 1
        
          color_red = ColorRGBA()
          color_red.r = 1
          color_red.g = 0
          color_red.b = 0
          color_red.a = 1
          color_blue = ColorRGBA()
          color_blue.r = 0
          color_blue.g = 0
          color_blue.b = 1
          color_blue.a = 1

          p = Point()
          p.x = self.current_pose[0]
          p.y = self.current_pose[1]
          p.z = 0.0
          m.points.append(p)
          m.colors.append(color_blue)
        
          for n in path:
              p = Point()
              p.x = n[0]
              p.y = n[1]
              p.z = 0.0
              m.points.append(p)
              m.colors.append(color_red)
        
          self.marker_pub.publish(m)
        
  # Publish a sample points in blue
  def publish_points(self):
      #print("Publish points")
      if len(self.random_points) > 0:
          print("Publish points!")
          m = Marker()
          m.header.frame_id = 'world_ned'
          m.header.stamp = rospy.Time.now()
          m.id = 1
          m.type = Marker.POINTS
          m.ns = 'points'
          m.action = Marker.DELETE
          m.lifetime = rospy.Duration(0)
          self.points_pub.publish(m)

          m.action = Marker.ADD
          m.scale.x = 0.05
          m.scale.y = 0.05
          m.scale.z = 0.05
        
          m.pose.orientation.x = 0
          m.pose.orientation.y = 0
          m.pose.orientation.z = 0
          m.pose.orientation.w = 1
        
          color_red = ColorRGBA()
          color_red.r = 1
          color_red.g = 0
          color_red.b = 0
          color_red.a = 1
          color_blue = ColorRGBA()
          color_blue.r = 0
          color_blue.g = 0
          color_blue.b = 1
          color_blue.a = 1

          for n in self.random_points:
              p = Point()
              p.x = n[0]
              p.y = n[1]
              p.z = 0.0
              m.points.append(p)
              m.colors.append(color_blue)
          #print(m)
          self.points_pub.publish(m)

  # Publish the goal point in green
  def publish_goal(self):
      #print("Publish points")
      if self.best_target_point is not None:
        print("Publish goal!")
        m = Marker()
        m.header.frame_id = 'world_ned'
        m.header.stamp = rospy.Time.now()
        m.id = 1
        m.type = Marker.POINTS
        m.ns = 'goal'
        m.action = Marker.DELETE
        m.lifetime = rospy.Duration(0)
        self.best_pub.publish(m)

        m.action = Marker.ADD
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
    
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
    
        color_green = ColorRGBA()
        color_green.r = 0
        color_green.g = 1
        color_green.b = 0
        color_green.a = 1
        p = Point()
        p.x = self.best_target_point[0]
        p.y = self.best_target_point[1]
        p.z = 0.0
        m.points.append(p)
        m.colors.append(color_green)

        self.best_pub.publish(m)

# ==================== MAIN FUNCTION ====================
if __name__ == '__main__':
  rospy.init_node('turtlebot_online_path_planning_node') 
  node = OnlinePlanner('/projected_map', '/turtlebot/kobuki/ground_truth', '/turtlebot/twist', np.array([-6.0, 6.0]), 0.3)
  node.explore()
  # Run forever
  rospy.spin()
