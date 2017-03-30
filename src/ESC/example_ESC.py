#!/usr/bin/python

import time
from ESC import ESC

# ===========================================================================
# Example code 
# ===========================================================================

# Initialisation 
# ESC(address=0x40, channel=0, freq=50)
esc = ESC(0x40, 15, 50)
esc.setPulseLengthMin(1000)
esc.setPulseLengthMax(2000)
'''
print "ESC calibration (~4sec)"
esc.setSpeed(100)
print "Maximum speed (2000us)"
time.sleep(2)
'''
esc.setSpeed(0)
print "Minimum speed (1000us)"
time.sleep(2)
# Running the motor
i=1
while i:
	i=0
	'''
	print "Speed: 10%"
	esc.setSpeed(10)
	time.sleep(0.5)
	'''
	print "Speed: 20%"
        esc.setSpeed(23)
        time.sleep(15)
	'''
	print "Speed: 50%"
        esc.setSpeed(50)
        time.sleep(2)
	
	print "Speed: 75%"
        esc.setSpeed(75)
        time.sleep(2)
	print "Speed: 100%"
	esc.setSpeed(100)
	time.sleep(3)
	print "Speed: 50%"
        esc.setSpeed(50)
        time.sleep(1)
	'''
	print "Speed: 0%"
        esc.setSpeed(0)
        time.sleep(4)
	

