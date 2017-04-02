#!/usr/bin/python

import sys
import time
import RPi.GPIO as GPIO


switch1_pin = 24
switch2_pin = 7

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setup(switch1_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch2_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

switch1 = False
switch2 = False

if (GPIO.input(switch1_pin) and GPIO.input(switch2_pin)):
    	pressed = "both"
	print "BOTH PRESSED"
elif (GPIO.input(switch1_pin)):
    	pressed = "sw1"
	print "SW1 PRESSED"
elif (GPIO.input(switch2_pin)):
    	pressed = "sw2"
	print "SW2 PRESSED"
else:
    	pressed = "none"
	print "NONE PRESSED"


# Main program loop.
while True:
	
	if (GPIO.input(switch1_pin)):
    		switch1 = True
    	else:
    		switch1 = False
	if (GPIO.input(switch2_pin)):
    		switch2 = True
    	else:
    		switch2 = False
	
   	if (switch1==True and switch2==True and not(pressed=="both")):
    		pressed = "both"
		print "BOTH PRESSED"
	elif (switch1==True and switch2==False and not(pressed=="sw1")):
    		pressed = "sw1"
		print "SW1 PRESSED"
	elif (switch1==False and switch2==True and not(pressed=="sw2")):
    		pressed = "sw2"
		print "SW2 PRESSED"
	elif (switch1==False and switch2==False and not(pressed=="none")):
    		pressed = "none"
		print "NONE PRESSED"
	
    	time.sleep(0.005)

