#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class Twist2Wheels:
    """
    Class converts Twist message containing linear and angular velocities to wheel left and right velocities.
    """

    def __init__(self):
        # Robot physical parameters
        self.b = 0.23  # wheelbase
        self.r = 0.035  # wheel radius

        # Twist messages subscriber 
        self.twist_sub = rospy.Subscriber('/turtlebot/twist', Twist, self.twist_callback)

        # Wheels velocities publisher
        self.wheel_pub = rospy.Publisher('/turtlebot/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)
        
    def twist_callback(self, msg):
        """
        Converts Twist message to left and right wheel velocities
        It also publishes the right and left velocities
        """
        v = msg.linear.x
        w = msg.angular.z
        
	# print the subscribed velocities
        print(f"linear velocity : {v}, angular velocity : {w}")

	# Compute linear velocities
        right_lin_vel = v + (w * self.b / 2.0)
        left_lin_vel = v - (w * self.b / 2.0)
        
        
	# Compute wheel seperate velocities
        right_wheel_vel = right_lin_vel / self.r
        left_wheel_vel = left_lin_vel / self.r
        
        # print the computed left and right wheel velocities
        print(f"left velocity : {left_wheel_vel}, right velocity : {right_wheel_vel}")


        # Publish wheel velocities
        self.wheel_pub.publish(Float64MultiArray(data = [right_wheel_vel,left_wheel_vel]))


if __name__ == '__main__':
    rospy.init_node('twist_vel')
    robot = Twist2Wheels()
    rospy.spin()

