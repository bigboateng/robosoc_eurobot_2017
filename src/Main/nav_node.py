#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from MotorControl.MD25 import MD25 
controller = MD25(0x58,1,True)

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
		leftEncoder, rightEncoder = controller.getEncoderValues()
		rospy.loginfo("Left={},right={}".format(leftEncoder,rightEncoder))
		rate.sleep()
		

if __name__ == "__main__":
	try:
		update_ukf()
	except rospy.ROSInterruptException:
		pass
