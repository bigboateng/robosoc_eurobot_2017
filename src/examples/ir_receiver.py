import rospy
from std_msgs.msg import String

def computePath(x):
	num = int(x.data)
	print(num)

def receiveSensorData():
	rospy.init_node("receiver_node", anonymous=True)
	rospy.Subcriber("sensorPublisher", String, computePath)
	rospy.spin()


if __name__ == "__main__":
	receiveSensorData()