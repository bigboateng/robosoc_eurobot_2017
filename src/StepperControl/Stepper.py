#!/usr/bin/python
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

# Define advanced sequence
# as shown in manufacturers datasheet
Seq = [[1,0,0,1],
       [1,0,0,0],
       [1,1,0,0],
       [0,1,0,0],
       [0,1,1,0],
       [0,0,1,0],
       [0,0,1,1],
       [0,0,0,1]]
        
StepCount = len(Seq)

# Initialise variables
StepCounter1 = 0
StepCounter2 = 0

StepperPins1 = [0,0,0,0]
StepperPins2 = [0,0,0,0]
WaitTime = 1/float(600)
StepperDir1 = 1
StepperDir2 = 1
TotalSteps = 400

def setupStepper(index, pins, totalSteps=400): 
	global StepperPins1
	global StepperPins2
	global TotalSteps

	if (index==1):
		StepperPins1 = list(pins)
	elif (index==2):
		StepperPins2 = list(pins)
	else:
		print "Error: the index must be 1 or 2"

	TotalSteps = totalSteps

	# Use BCM GPIO references
	# instead of physical pin numbers
	GPIO.setmode(GPIO.BCM)
	
	# Set all pins as output
	for pin in pins:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin, False)
  
def setSpeed(speed=600): # steps/second
	global WaitTime
	WaitTime = 1/float(speed)

def reverseDirBoth():
	global StepperDir1
	global StepperDir2
	StepperDir1 = -StepperDir1
	StepperDir2 = -StepperDir2

def reverseDir(index):
	global StepperDir1
	global StepperDir2

	if (index==1):
		StepperDir1 = -StepperDir1
	elif (index==2):
		StepperDir2 = -StepperDir2
	else:
		print "Error: the index must be 1 or 2"

def setDirBoth(direction):
	global StepperDir1
	global StepperDir2

	if (direction==0 | direction<-2 | direction>2):
		print "Error: the direction must be 1, -1, 2 or -2"

	StepperDir1 = direction
	StepperDir2 = direction

def setDir(index, direction):
	global StepperDir1
	global StepperDir2
	
	if (direction==0 | direction<-2 | direction>2):
		print "Error: the direction must be 1, -1, 2 or -2"

	if (index==1):
		StepperDir1 = direction
	elif (index==2):
		StepperDir2 = direction
	else:
		print "Error: the index must be 1 or 2"

def move_step(index, steps=0):
	global StepperPins1
	global StepperPins2
	global Seq
	global StepCounter1
	global StepCounter2
	global StepperDir1
	global StepperDir2
	global StepCount
	global WaitTime
  	
	if (index==1):
		StepperPins = list(StepperPins1)
		StepCounter = StepCounter1
		StepperDir = StepperDir1
	elif (index==2):
		StepperPins = list(StepperPins2)
		StepCounter = StepCounter2
		StepperDir = StepperDir2
	else:
		print "Error: the index must be 1 or 2"
	
	for index in range(steps):
		steps -= 1
		for pin in range(0,4):
			xpin=StepperPins[pin]
			if Seq[StepCounter][pin]!=0:
				GPIO.output(xpin, True)
			else:
				GPIO.output(xpin, False)
	  
		StepCounter += StepperDir
 
		# If we reach the end of the sequence
		# start again
		if (StepCounter>=StepCount):
			StepCounter = 0
		if (StepCounter<0):

			StepCounter = StepCount+StepperDir
 
		# Wait before moving on
		time.sleep(WaitTime) #0.01/12

def moveBoth_step(steps=0):
	global StepperPins1
	global StepperPins2
	global Seq
	global StepCounter1
	global StepCounter2
	global StepperDir1
	global StepperDir2
	global StepCount
	global WaitTime
  
	for index in range(steps):
		steps -= 1
		for pin in range(0,4):
			xpin=StepperPins1[pin]
			if Seq[StepCounter1][pin]!=0:
				GPIO.output(xpin, True)
			else:
				GPIO.output(xpin, False)
 
		for pin in range(0,4):
			xpin=StepperPins2[pin]
			if Seq[StepCounter2][pin]!=0:
				GPIO.output(xpin, True)
			else:
				GPIO.output(xpin, False)
	  
		StepCounter1 += StepperDir1
 		StepCounter2 += StepperDir2

		# If we reach the end of the sequence
		# start again
		if (StepCounter1>=StepCount):
			StepCounter1 = 0
		if (StepCounter1<0):
			StepCounter1 = StepCount+StepperDir1
		
		if (StepCounter2>=StepCount):
			StepCounter2 = 0
		if (StepCounter2<0):
			StepCounter2 = StepCount+StepperDir2
 
		# Wait before moving on
		time.sleep(WaitTime) #0.01/12
	
def move_revo(index, revolutions=0):
	global TotalSteps
	steps = int(revolutions*TotalSteps)
	move_step(index, steps)

def moveBoth_revo(revolutions=0):
	global TotalSteps
	steps = int(revolutions*TotalSteps)
	moveBoth_step(steps)


