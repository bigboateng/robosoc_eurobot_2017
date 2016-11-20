import rospy
from std_msgs.msg import String



def sendSensorData():
	rospy.init_node("sensor_node", anonymous=True)
	rate = rospy.Rate(5)
	sensorPublisher = rospy.Publisher("sensorPublisher", String, queue_size=10)
	number = 10
	while not rospy.is_shutdown():
		sensorPublisher.Publisher(str(number))
		rate.sleep()




if __name__ == "__main__":
	try:
		sendSensorData()
	except rospy.ROSInterruptException:
		pass