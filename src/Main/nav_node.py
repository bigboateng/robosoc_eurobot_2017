#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def update_ukf():
	# Function to send pose data to ukf node

	# init node
	rospy.init_node("nav_node", anonymous=True)
	# update every 1 second
	rate = rospy.Rate(1)
	# odom publisher
	odom_publisher = rospy.Publisher("odometry_publisher", String, queue_size=10)
	count = 0
	while not rospy.is_shutdown():
		count = count + 1
		rospy.loginfo(str(count))
		rate.sleep()
		

if __name__ == "__main__":
	try:
		update_ukf()
	except rospy.ROSInterruptException:
		pass
