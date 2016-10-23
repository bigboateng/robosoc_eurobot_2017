#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Rate to update code (1Hz)
rate = None
# Time limit for competition 90 sec
max_time = 10

def initialize():
	global rate
	# Initialize Node 
	rospy.init_node("timer_node", anonymous=True)
	# Initialize publisher
	rospy.Publisher("elapsed_time", String, queue_size=10)
	# Rate to update code (1Hz)
	rate = rospy.Rate(1)
	update_time()



def update_time():
	global rate, max_time
	start_time = rospy.get_time()
	while not rospy.is_shutdown():
		curr_time = rospy.get_time()
		if curr_time - start_time > max_time:
			print("Time limit exceed")
			break
		else:
			rospy.loginfo("Time elapsed: {}".format(curr_time - start_time))
		rate.sleep()


if __name__ == "__main__":
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass