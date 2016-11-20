#!/usr/bin/env python

import rospy
from std_msgs.msg import String

velocityPublisher = None
def computePath(x):
    global velocityPublisher
    num = int(x.data)
    velocityPublisher.publish("Message Receibed!")	 
    print(num*2)


def init():
    global velocityPublisher
    print("Initializing Programme")
    rospy.init_node("Number_Receiver", anonymous=True)
    path_sub = rospy.Subscriber("Position", String, computePath)
    velocityPublisher = rospy.Publisher("velocity", String, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
	rate.sleep()

if __name__ == "__main__":
    try:
	init()
    except rospy.ROSInterruptException:
        pass	    
