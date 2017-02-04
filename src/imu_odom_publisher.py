#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2

# import BN005 library and non-ros stuff
from Adafruit_BNO055 import BNO055
import math
# Global variables
rate = None
imu_publisher = None
current_time = None
last_time = None

# Imu setup
bno = BNO055.BNO055(rst=7)
default_refresh_rate = 5
log_data = False

# Global robot position
robot_pos_x = 0
robot_pos_y = 0
robot_theta = 0

def toRadsPerSec(val):
	return val * 0.0174533


def convertAngularRatesToRadians(data):
	# This function converts angular velocities/accelerations from degs/sec to rads/sec
	x_degs , y_degs , z_degs = data
	if log_data:
		print("x_rads = {}, y_rads= {}, z_rads={}".format(x_degs, y_degs, z_degs))
	x_rads = toRadsPerSec(x_degs)
	y_rads = toRadsPerSec(y_degs)
	z_rads = toRadsPerSec(z_degs)
	return x_rads, y_rads, z_rads

def convertEurlerAnglesToRads(data):
	heading, roll, pitch = data
	return math.radians(heading), math.radians(roll), math.radians(pitch)

def init():
	global rate, current_time, last_time
	global imu_publisher
	# Initialize the node
	rospy.init_node("imu_odom_node", anonymous=True)
	# Refresh at 10Hz	
	rate = rospy.Rate(default_refresh_rate)
	# Check Imu is connected
	if not bno.begin():
		raise RuntimeError("Failed to initalize IMU, is reset pin 7?")
	
	# Imu publisher (for robot_localization node)
	imu_publisher = rospy.Publisher("Imu_publisher", Imu, queue_size=50)
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	# Main loop
	run_main_program()

def run_main_program():
	global rate, imu_publisher
	while not rospy.is_shutdown():
		# First read imu data
		heading, roll, pitch = convertEurlerAnglesToRads(bno.read_euler())
 		# Quaternion data
		x, y, z, w = bno.read_quaternion()
		# Angular velocity data in rad/sec
		ang_x, ang_y, ang_z = convertAngularRatesToRadians(bno.read_gyroscope())
		# Linear acceleration m/s^2
		accel_x, accel_y, accel_z  = bno.read_linear_acceleration()
		# create Imu object
		imu = Imu()
		imu.header.stamp = current_time
		imu.orientation.x = x
		imu.orientation.y = y
		imu.orientation.z = z
		imu.orientation.w = w
		imu.angular_velocity.x = ang_x
		imu.angular_velocity.y = ang_y
		imu.angular_velocity.z = ang_z
		imu.linear_acceleration.x = accel_x
		imu.linear_acceleration.y = accel_y
		imu.linear_acceleration.z = accel_z
		imu_publisher.publish(imu)

		# Create odometry object
		
		if log_data:
			print("Heading={0:0.2F}, Roll={1:0.02F}, Pitch={2:0.2F}".format(heading, roll, pitch))	
		print("I am working")

		last_time = current_time
		rate.sleep()

if __name__ == "__main__":
	try:
		init()
	except rospy.ROSInterruptException:
		pass
