#!/usr/bin/python

import time
from Servo import Servo

# ===========================================================================
# Code to grab a cyclinder with the arm
# ===========================================================================

# Initialisation 
# Main servo (HX12K)
servo_main = Servo(0x40, 0, 50)
servo_main.setPulseLengthMin(550)
servo_main.setPulseLengthMax(2450)
servo_main.setMaxAngle(180)
# Secondary servo (SM-S2309S)
servo_sec = Servo(0x40, 1, 50)
servo_sec.setPulseLengthMin(550)
servo_sec.setPulseLengthMax(2450)
servo_sec.setMaxAngle(180)

# Grabs a cyclinder
# Note : angles are anti-clockwise
print "Angle2: 40 deg."
servo_sec.setAngle(40)
time.sleep(1)

print "Angle1: -88 deg."
servo_main.setAngle(-88)
time.sleep(1)

print "Angle2: -90 deg."
servo_sec.setAngle(-90)
time.sleep(1)
	
print "Angle2: 40 deg."
servo_sec.setAngle(40)
time.sleep(1)

#print "Angle1: 12 deg."
print "Angle1: 7 deg."
servo_main.setAngle(7)
time.sleep(1.5)

print "Angle2: -90 deg."
servo_sec.setAngle(-90)
time.sleep(1)

print "Angle2: 40 deg."
servo_sec.setAngle(40)
time.sleep(1)
print "Done"

