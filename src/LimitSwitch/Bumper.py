#!/usr/bin/python

import sys
import RPi.GPIO as GPIO

# ===========================================================================
# Bumper Class
# ===========================================================================

class Bumper:

  # Private Fields

  # Constructor
  def __init__(self, gpio_pin=[], pull_resistor="PULL_OFF", debug=False):
	# Check input parameters
	if (pull_resistor=="PULL_DOWN"):
		pull_up_down=GPIO.PUD_DOWN
	elif (pull_resistor=="PULL_UP"):
		pull_up_down=GPIO.PUD_UP
	elif (pull_resistor=="PULL_OFF"):
		pull_up_down=GPIO.PUD_OFF
	else:
		sys.exit("Error: The pull_resistor parameter must be PULL_DOWN, PULL_UP or PULL_OFF")
	
	if (len(gpio_pin)<1):
		sys.exit("Error: The list of input GPIOs is empty")
	
	# Initialise GPIOs
	GPIO.setmode(GPIO.BCM) # Use BCM GPIO references instead of physical pin numbers
	for i in range(len(gpio_pin)):
		GPIO.setup(gpio_pin[i], GPIO.IN, pull_up_down)
	
	# Initialise private variables
	self.pin = list(gpio_pin)

	self.debug = debug 


  def changePullRes(self, gpio_pin, pull_resistor="PULL_OFF"):
	if (gpio_pin in self.pin):
		if (pull_resistor=="PULL_DOWN"):
			GPIO.setup(gpio_pin, GPIO.IN, GPIO.PUD_DOWN)
		elif (pull_resistor=="PULL_UP"):
			GPIO.setup(gpio_pin, GPIO.IN, GPIO.PUD_UP)
		elif (pull_resistor=="PULL_OFF"):
			GPIO.setup(gpio_pin, GPIO.IN, GPIO.PUD_OFF)
		else:
			sys.exit("Error: The pull_resistor parameter must be PULL_DOWN, PULL_UP or PULL_OFF")
	else:
		sys.exit("Error: Wrong GPIO pin given")

  def getInput(self, gpio_pin):
	if (gpio_pin in self.pin):
		return GPIO.input(gpio_pin)
	else:
		sys.exit("Error: Wrong GPIO pin given")
	

  def areTrue(self, gpio_pin=[]):
	if (len(gpio_pin)>1):
		for i in range(len(gpio_pin)):
			if (gpio_pin[i] in self.pin):
				if(not( GPIO.input(gpio_pin[i]) )):
					return False
			else:
				sys.exit("Error: Wrong GPIO pin given")
			
		return True
	else:
		gpio_pin = list(self.pin)
		for i in range(len(gpio_pin)):
			if (gpio_pin[i] in self.pin):
				if(not( GPIO.input(gpio_pin[i]) )):
					return False
			else:
				sys.exit("Error: Wrong GPIO pin given")
			
		return True

	

