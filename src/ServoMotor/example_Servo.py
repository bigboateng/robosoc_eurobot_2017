#!/usr/bin/python

import time
from Servo import Servo

# ===========================================================================
# Example code using an SG90 servo
# ===========================================================================

# Initialisation (for an SG90 servo)
servo_SG90 = Servo(0x40, 0, 50)
servo_SG90.setPulseLengthMin(550)
servo_SG90.setPulseLengthMax(2450)
servo_SG90.setMaxAngle(180)

# Moving servo on channel 0
while 1:
	# Note : angles are anti-clockwise
	print "Angle: 45 deg."
	servo_SG90.setAngle(45)
	time.sleep(1)
	print "Angle: 90 deg."
	servo_SG90.setAngle(90)
	time.sleep(1)
	print "Angle: -90 deg."
	servo_SG90.setAngle(-90)
	time.sleep(1)
	print "Angle: 0 deg."
	servo_SG90.setAngle(0)
	print "Done"
	time.sleep(2)

