#!/usr/bin/python

import time
from Bumper import Bumper

# ===========================================================================
# Example code for Bumper Class
# ===========================================================================

pins = [24,7]
bumper = Bumper(pins, "PULL_DOWN")
pressed = False
count = 0

while 1:
	if (bumper.areTrue() and pressed==False):
		pressed = True
		count += 1
		print "BOTH PRESSED, %d" % (count)
	elif (not(bumper.areTrue()) and pressed==True):
		pressed = False
		print "NOT BOTH PRESSED"
	time.sleep(0.001)
	
