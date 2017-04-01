#!/usr/bin/env python

import time
from os import sys, path
sys.path.append('../UltrasonicSensors')
from SRF08 import SRF08
sys.path.append('../IRSensors')
from IRsensor import IRsensor
sys.path.append('../ESC')
from ESC import ESC
sys.path.append('../StepperControl')
import Stepper

# Parameters

# SPI/IR Sensor
SPI_PORT   = 0
SPI_DEVICE = 0
channel_mcp_IR_sensor_right = 0
channel_mcp_IR_sensor_left = 1

# Ultrasonic Sensor
address_i2c_US_sensor_right = 0xE0 >> 1
address_i2c_US_sensor_left = 0xE2 >> 1 # CHANGE ADDRESS

# PCA9685
address_i2c_PCA9685 = 0x40
# ESC
channel_PCA9685_ESC = 15
esc.setPulseLengthMin(1000)
esc.setPulseLengthMax(2000)

# Stepper motors
rightStepper = 1 
leftStepper = 2 
stepperPins1 = [17,22,23,24] # Pins of right stepper
stepperPins2 = [5,6,13,19] # Pins of left stepper
totalSteps = 4076
speed = 1200 # 1200 steps/sec max @5V
# Lift strategy
revo_tilt_1 = 0.4 # 1) Tilt the plate a bit
revo_lift_1 = 0.3 # 2) Lift the plate a bit
revo_tilt_2 = 0.3 # 3) Tilt the plate even more
revo_lift_2 = 0.7 # 4) Lift the plate to the top

side = 'yellow' # blue side / 2: yellow side
if(side=='blue'):
	motor_tilt = rightStepper # Right stepper
elif(side=='yellow'):
	motor_tilt = leftStepper # Left stepper
else:
	sys.exit("Error: The side parameter must be 'blue' or 'yellow'")


# Initialisation 

# IR Sensor RIGHT
IR_sensor_right = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor_right)
# IR Sensor LEFT
IR_sensor_left = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor_left)

# Ultrasonic Sensor RIGHT
US_sensor_right = SRF08(address_US_sensor_right, 2)
# Ultrasonic Sensor LEFT
US_sensor_left = SRF08(address_US_sensor_left, 2) 

# ESC
esc = ESC(address_i2c_ESC, channel_PCA9685_ESC, 50)

# Stepper Motors
Stepper.setupStepper(rightStepper, stepperPins1, totalSteps)
Stepper.setupStepper(leftStepper, stepperPins2, totalSteps)
Stepper.setSpeed(speed) 
Stepper.setDir(rightStepper,-1)


# Functions

def getDistance_IRsensor():
    	global IR_sensor_right
    	global IR_sensor_left
    	IR_sensor_right.getAnalogValue()
    	IR_sensor_left.getAnalogValue()
    	IR_sensor_right.distance # output this to ROS
    	IR_sensor_left.distance # output this to ROS

def getDistance_USsensor():
    	global US_sensor_right
    	global US_sensor_left
    	US_sensor_right.readEcho(1)
    	US_sensor_left.readEcho(1)
    	US_sensor_right.echo[0] # output this to ROS
    	US_sensor_left.echo[0] # output this to ROS

def setSpeedESC(speed):
    	global esc
    	esc.setSpeed(speed)

def setBallGrabberOn(speed=28): # Rename ? + change speed
    	global esc
    	esc.setSpeed(speed) 

def setBallGrabberOff():
    	global esc
    	esc.setSpeed(0)

def liftUp():
	global motor_tilt
	global revo_lift_1
	global revo_lift_2
	global revo_tilt_1
	global revo_tilt_2
	Stepper.moveBoth_revo(revo_lift_1)
    	Stepper.move_revo(motor_tilt, revo_tilt_1)
    	Stepper.moveBoth_revo(revo_lift_2)
    	Stepper.move_revo(motor_tilt, revo_tilt_2)

def liftDown():
	global motor_tilt
	global revo_lift_1
	global revo_lift_2
	global revo_tilt_1
	global revo_tilt_2
    	Stepper.reverseDirBoth()
    	Stepper.moveBoth_revo(revo_lift_1)
    	Stepper.moveBoth_revo(revo_lift_2)
    	Stepper.move_revo(motor_tilt, revo_tilt_1)
    	Stepper.move_revo(motor_tilt, revo_tilt_2)

def liftSequence(sleep_time=2):
	global motor_tilt
	global revo_lift_1
	global revo_lift_2
	global revo_tilt_1
	global revo_tilt_2
	Stepper.moveBoth_revo(revo_lift_1)
    	Stepper.move_revo(motor_tilt, revo_tilt_1)
    	Stepper.moveBoth_revo(revo_lift_2)
    	Stepper.move_revo(motor_tilt, revo_tilt_2)
    	time.sleep(sleep_time)
    	Stepper.reverseDirBoth()
    	Stepper.moveBoth_revo(revo_lift_1)
    	Stepper.moveBoth_revo(revo_lift_2)
    	Stepper.move_revo(motor_tilt, revo_tilt_1)
    	Stepper.move_revo(motor_tilt, revo_tilt_2)




