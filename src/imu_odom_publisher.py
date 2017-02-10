#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
#import tf and tf2
import tf
import tf2_ros
# import BN005 library and non-ros stuff
from Adafruit_BNO055 import BNO055
# import motor library
import sys
import os.path
sys.path.append("/home/pi/ros_catkin_ws/src/robosoc_eurobot_2017/src/MotorControl")
from MD25 import MD25
from Robot import Robot
import math
import time
# Global variables
rate = None
imu_publisher = None
odom_publisher = None
current_time = None
last_time = None

# Imu setup
bno = BNO055.BNO055(rst=7)
default_refresh_rate = 1
log_data = True
has_set_yaw_offset = False
# setup primary robot
secondary_robot = Robot(axle_length = 0.181, wheel_diameter=0.1)

# md25 setup 
motor_controller = MD25(address=0x58, mode=1, debug=True, robot=secondary_robot)
motor_controller.resetEncoders()
# Tf2 odometry broadcaster 
odom_transform_broadcaster = tf2_ros.TransformBroadcaster()



def toRadsPerSec(val):
	return val * 0.0174533

def convertAngularRatesToRadians(data):
	# This function converts angular velocities/accelerations from degs/sec to rads/sec
	x_degs , y_degs , z_degs = data
	#if log_data:
	#	print("x_rads = {}, y_rads= {}, z_rads={}".format(x_degs, y_degs, z_degs))
	x_rads = toRadsPerSec(x_degs)
	y_rads = toRadsPerSec(y_degs)
	z_rads = toRadsPerSec(z_degs)
	return x_rads, y_rads, z_rads

def convertEurlerAnglesToRads(data):
	heading, roll, pitch = data
	return math.radians(heading), math.radians(roll), math.radians(pitch)


def move_forward():
	controller.forward()
	controller.stop()

def init():
	global bno
	global rate, current_time, last_time
	global imu_publisher, odom_publisher
	# Initialize the node
	rospy.init_node("imu_odom_node", anonymous=True)
	# Refresh at 10Hz	
	rate = rospy.Rate(default_refresh_rate)
	# Check Imu is connected
	if not bno.begin():
		raise RuntimeError("Failed to initalize IMU, is reset pin 7?")
	# Set yaw offset
	# take 10 readings then set off set of yaw		
	#heading, roll, pitch = convertEurlerAnglesToRads(bno.read_euler())
	#yaw_offset = heading
	#print("Intital Heading={:.2f}, roll={:.2f}, yaw={:.2f}".format(heading, roll,pitch))
	#rospy.loginfo("Yaw offset = {}".format(yaw_offset))
	#rospy.set_param("yaw_offset", yaw_offset)
	# Imu publisher (for robot_localization node)
	imu_publisher = rospy.Publisher("imu/data", Imu, queue_size=50)
	odom_publisher = rospy.Publisher("odom/data", Odometry, queue_size=50)
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	# Main loop
	rospy.loginfo("Main program started")
	run_main_program()

def run_main_program():
	global rate, imu_publisher, odom_publisher, bno, has_set_yaw_offset
	# set yaw offset
	heading, roll, pitch = convertEurlerAnglesToRads(bno.read_euler())
	while not rospy.is_shutdown():
		# set current time
		current_time = rospy.Time.now()
		# First read imu data
		heading, roll, pitch = convertEurlerAnglesToRads(bno.read_euler())
 		if not has_set_yaw_offset:
			if heading > 0.0 < 20:
				rospy.set_param("yaw_offset", heading)
				print("Yaw offset = {:.2f}".format(heading))
				has_set_yaw_offset = True
		else:
			 # set current time
                current_time = rospy.Time.now()
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

			if log_data:
				print("Heading={:0.2f}, Roll={:0.02f}, Pitch={:.2f}".format(heading, roll, pitch))

			# Generate odometry data for publishing over tf
			odom_trans = TransformStamped()
			odom_trans.header.stamp = current_time
			odom_trans.header.frame_id = "odom"
			odom_trans.child_frame_id = "base_link"

			# update the position
			motor_controller.updatePosition()

			odom_trans.transform.translation.x = motor_controller.getXPosition()
			odom_trans.transform.translation.y = motor_controller.getYPosition()
			odom_trans.transform.translation.z = 0.0
			q = tf.transformations.quaternion_from_euler(0,0, motor_controller.getTheta())
			odom_trans.transform.rotation = q	

			# Send the transform
			odom_transform_broadcaster.sendTransform(odom_trans)

			# Next publish odometry info over ros to robot_localization
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"
			odom.child_frame_id = "base_link"
		
			# set the position
			odom.pose.pose.position.x = motor_controller.getXPosition()
			odom.pose.pose.position.y = motor_controller.getYPosition()
			odom.pose.pose.position.z = 0.0
			odom_publisher.publish(odom)
		
			# odom readings in node 
			rospy.loginfo("x={}, y={}, theta={}".format(motor_controller.getXPosition(),motor_controller.getYPosition(), motor_controller.getTheta()))
		last_time = current_time
		rate.sleep()

if __name__ == "__main__":
	try:
		init()
	except rospy.ROSInterruptException:
		pass
