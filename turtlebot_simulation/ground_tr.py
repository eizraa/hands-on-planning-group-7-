#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Basic imports
import numpy as np
import rospy
import tf

# from tf.broadcaster import _
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# ROS messages
from nav_msgs.msg import Odometry

class Robot:
    def __init__(self):

        # Subscriber to get the ground truth
        self.odom_pub = rospy.Subscriber(
            '/turtlebot/kobuki/ground_truth', Odometry, self.odom_path_pub)
        self.tf_br = tf.TransformBroadcaster()
   
    def odom_path_pub(self,odom):
        # Transform theta from euler to quaternion
        q = [odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w]
            
	 # Set the fram ID as world_ned
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "turtlebot/kobuki/base_footprint"


        self.tf_br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0), q, rospy.Time.now(
        ), odom.child_frame_id, odom.header.frame_id)


if __name__ == '__main__':
    rospy.init_node('Ground_tr')
    robot = Robot()
    rospy.spin()
