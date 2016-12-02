#!/usr/bin/env python
# import ros system
import rospy
# import BN055 library
from Adafruit_BNO055 import BNO055

# Create and configure the BNO sensor connection.
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
bno = BNO055.BNO055(rst="P9_19")

def initializeSensor():
	if not bno.begin():
		return False # Failed to initialize
	else:
		return True # Sucessfullt Initialized

def printIMUDiagnosticData():
	sw, bl, accel, mag, gyro = bno.get_revision()
	print('Software version:   {0}'.format(sw))
	print('Bootloader version: {0}'.format(bl))
	print('Accelerometer ID:   0x{0:02X}'.format(accel))
	print('Magnetometer ID:    0x{0:02X}'.format(mag))
	print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


def startNavigationSystem(hertz=1):  # start taking readings at 10 hz (default)
	pitch, yaw, roll = 0.0
	pitch_dot, yaw_dot, roll_dot = 0.0
	while not rospy.is_shutdown():
		sys, gyro, accel, mag = bno.get_calibration_status()
	    # Print everything out.
	    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
	          heading, roll, pitch, sys, gyro, accel, mag))
		rate.sleep(hertz)

if __name__ == "__main__":
	try:
		startNavigationSystem(20)
	except rospy.ROSInterruptExecption:
		pass