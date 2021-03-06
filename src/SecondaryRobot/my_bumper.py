#!/usr/bin/env python
"""
This node listens for button clicks and alert the main program
"""
import rospy
from std_msgs.msg import String
from Bumper import Bumper
from PrimaryRobotState import BumperType


refresh_rate = 10 # 10 times a second


def init_node():
	global refresh_rate
	rospy.init_node("bumperr", anonymous=True)
	rate = rospy.Rate(refresh_rate)
	# Bumper pins
	pins = [17,27]
	grabber_bumber = Bumper([22], "PULL_DOWN")
	moon_base_bumper = Bumper(pins, "PULL_DOWN")
	start_button_bumber = Bumper([4], "PULL_UP")
	# publisher to alert when bumper is pressed
	bumper_pub = rospy.Publisher("bumper_topic", String, queue_size=10)
	rospy.loginfo("Beginning node")
	while not rospy.is_shutdown():
		if grabber_bumber.isPressed():
			bumper_pub.publish(BumperType.GRABBER_BUMPER)
			rospy.loginfo("GRABBER BUMPER PRESSED")

		if moon_base_bumper.isPressed():
			rospy.loginfo("BOTTOM PRESSED")
			bumper_pub.publish(BumperType.MOON_BASE_BUMPER)

		# if start_button_bumber.isPressed():
		# 	pass
		# 	bumper_pub.publish(BumperType.START_BUMPER)
		rate.sleep()


if __name__ == "__main__":
	try:
		init_node()
	except rospy.ROSInterruptException:
		pass
