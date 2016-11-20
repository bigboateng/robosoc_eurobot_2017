#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
import datetime


def printVelocity(msg):
    print(msg.data)


def talker():
    print("Inititalizing publishser...")
    rospy.init_node("Number_Sender", anonymous=True)
    rate = rospy.Rate(2)
    #node = rospy.init_node("node", anonymous=True)
    timePublisher = rospy.Publisher("Position", String, queue_size=10)
    velocitySub = rospy.Subscriber("velocity", String, printVelocity)
    while not rospy.is_shutdown():
	number = str(1)
	timePublisher.publish(number)
	print(number)
	rate.sleep()



if __name__ == "__main__":
    try:
	talker()
    except rospy.ROSInterruptException:
	pass
	
