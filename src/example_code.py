#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def computePath(x):
    print(x.data)


def init():
    print("Initializing Programme")
    rospy.init_node("node", anonymous=True)
    path_sub = rospy.Subscriber("node", String, computePath)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
	rate.sleep()

if __name__ == "__main__":
    try:
	init()
    except rospy.ROSInterruptException:
        pass	    
