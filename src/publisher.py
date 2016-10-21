#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
import datetime

def talker():
    print("Inititalizing publishser...")
    rospy.init_node("node", anonymous=True)
    rate = rospy.Rate(2)
    #node = rospy.init_node("node", anonymous=True)
    timePublisher = rospy.Publisher("node", String, queue_size=10)
    while not rospy.is_shutdown():
	curr_time = str(rospy.get_time())
	timePublisher.publish(curr_time)
	rospy.loginfo(curr_time)
	rate.sleep()



if __name__ == "__main__":
    try:
	talker()
    except rospy.ROSInterruptException:
	pass
	
