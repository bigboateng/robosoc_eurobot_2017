#!/usr/bin/env python
"""
This node listens for button clicks and alert the main program
"""
import rospy
from std_msgs.msg import String
from Bumper import Bumper
from PrimaryRobotState import BumperType


refresh_rate = 1 # 10 times a second


def init_node():
	global refresh_rate
	rospy.init_node("bumperr", anonymous=True)
	rate = rospy.Rate(refresh_rate)
	# Bumper pins
	pins = [24,7]
	grabber_bumber = Bumper([8], "PULL_DOWN")
	moon_base_bumper = Bumper(pins, "PULL_DOWN")
	# publisher to alert when bumper is pressed
	bumper_pub = rospy.Publisher("bumper_topic", String, queue_size=10)
	while not rospy.is_shutdown():
		rospy.loginfo("Working")
		if grabber_bumber.isPressed():
			bumber_pub.publish(BumperType.GRABBER_BUMPER)
		else:
			rospy.loginfo("grabber_bumper NOT pressed")

		if moon_base_bumper.isPressed():
			bumber_pub.publish(BumperType.MOON_BASE_BUMPER)
		rate.sleep()


if __name__ == "__main__":
	try:
		init_node()
	except rospy.ROSInterruptException:
		pass
