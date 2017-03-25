#!/usr/bin/python

import time
from Servo import Servo

# ===========================================================================
# Code to block the cyclinders
# ===========================================================================

# Initialisation 
# Servo blocking cylinders (SO5NF STD)
servo_cyl = Servo(0x40, 0, 50)
servo_cyl.setPulseLengthMin(550)
servo_cyl.setPulseLengthMax(2450)
servo_cyl.setMaxAngle(180)

# Initial position
# Note : angles are anti-clockwise
print "Angle: -90 deg."
servo_cyl.setAngle(-90)
time.sleep(5)
print "Angle: 35 deg."
servo_cyl.setAngle(35)
time.sleep(5)
print "Angle: 90 deg."
servo_cyl.setAngle(90)
print "Done"
time.sleep(2)

