#!/usr/bin/python

import sys
import time
import RPi.GPIO as GPIO


# ===========================================================================
# Encoder Class
# ===========================================================================
'''
class Encoder:

  # Private Fields
  count = 0

  # Constructor
  def __init__(self, gpio_pin, debug=False):
	# Use BCM GPIO references
	# instead of physical pin numbers
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
	self.pin = gpio_pin

	self.debug = debug # Make sure the specified mode

  def resetCount(self):
	count = 0
'''

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_OFF)


count = 0
if (GPIO.input(2)):
    state = 0
else:
    state = 1

# Main program loop.
while True:

    if (GPIO.input(2)):
    	encoder = True
    else:
    	encoder = False

    if (state==1 and encoder==True):
    	state = 0
	count = (count+1)%120
	print("{}".format(count))
    elif (state==0 and encoder==False):
	state = 1
	count = (count+1)%120
	print("{}".format(count))

    time.sleep(0.005)

