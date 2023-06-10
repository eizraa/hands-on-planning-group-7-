#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('myTestNode',anonymous=True)
    myString_pub = rospy.Publisher("/myToyexample",String, queue_size=1)
    rate = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():
        myString_pub.publish("Hello")
        rate.sleep()