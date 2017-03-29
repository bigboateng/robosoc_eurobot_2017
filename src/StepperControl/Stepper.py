#!/usr/bin/python

import sys
import threading
import time
import RPi.GPIO as GPIO

# ===========================================================================
# Stepper Class
# ===========================================================================

class Stepper:

  seq = [[1,0,0,1],
       [1,0,0,0],
       [1,1,0,0],
       [0,1,0,0],
       [0,1,1,0],
       [0,0,1,0],
       [0,0,1,1],
       [0,0,0,1]]
  stepCount = len(seq)
  #max_speed = 1200 # steps/second
  min_delay = (0.01*10000/12)
  waitTime = min_delay*100

  # Private Fields
  steps = 0
  stepCounter = 0
  stepSeqCounter = 0
  stepDir = 1 # Set to 1 or 2 for clockwise
              # Set to -1 or -2 for anti-clockwise

  # Constructor
  def __init__(self, stepPins=[17,22,23,24], totalSteps=4076, diameter_mm=5, debug=False):
	# Use BCM GPIO references instead of physical pin numbers
	self.stepPins = list(stepPins) 
	# Set all pins as output
	GPIO.setmode(GPIO.BCM)
	for pin in self.stepPins:
  	  GPIO.setup(pin,GPIO.OUT)
  	  GPIO.output(pin, False)
	self.totalSteps = totalSteps
	self.diameter_mm = diameter_mm

	thread = threading.Thread(target=self.run, args=())
	thread.daemon = True
	thread.start()

	self.debug = debug 

  def setSpeed(self, speed): 
	self.waitTime = self.min_delay*100 #TODO
  
  def changeDir(self, direction):
	if (direction == 1 | direction == -1) :
		self.stepDir = direction 
	else :
		print "Error: The direction parameter must be 1 or -1"
  
  def run(self):
	self.running = True
	self.move()
  
  def move_step(self, steps):
	self.steps = steps

  def move(self):
  	while self.running:
	  for index in range(self.steps):
	  	self.steps -= 1
	  	for pin in range(0,4):
    	    	xpin=self.stepPins[pin]# Get GPIO
    	    	if self.seq[self.stepCounter][pin]!=0:
      	      	  GPIO.output(xpin, True)
    	    	else:
      	      	  GPIO.output(xpin, False)
 
  	  	self.stepCounter += self.stepDir
 
  	  	# If we reach the end of the sequence
  	  	# start again
  	  	if (self.stepCounter>=self.stepCount):
    	    	  self.stepCounter = 0
  	  	if (self.stepCounter<0):
    	    	  self.stepCounter = self.stepCount+self.stepDir
 
  	  	# Wait before moving on
  	  	time.sleep(self.waitTime/1000000) 

'''
  def move_mm(self, millimeters):

  def move_rev(self, revolutions):

  def changeMode(self, mode):
'''







