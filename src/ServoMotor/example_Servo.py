#!/usr/bin/python

import time
from Servo import Servo

# ===========================================================================
# Example code using an SG90 servo
# ===========================================================================

# Initialisation (for an SG90 servo)
servo_SG90 = Servo(0x40, 50)
servo_SG90.setPulseLengthCenter(1500)
servo_SG90.setCoef(float(1000/90))

# Moving servo on channel 0
while 1:
	# Note : angles are counter-clockwise
	servo_SG90.setAngle(0, 45)
	time.sleep(1)
	servo_SG90.setAngle(0, 90)
	time.sleep(1)
	servo_SG90.setAngle(0, -90)
	time.sleep(1)
	servo_SG90.setAngle(0, 0)
	time.sleep(2)
	print "Done"

