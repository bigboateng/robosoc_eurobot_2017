#!/usr/bin/env python

from os import sys, path
sys.path.append('../UltrasonicSensors')
from SRF08 import SRF08
sys.path.append('../IRSensors')
from IRsensor import IRsensor
import time

# Parameters

# SPI/IR Sensor
SPI_PORT   = 0
SPI_DEVICE = 0
channel_mcp_IR_sensor_right = 0
channel_mcp_IR_sensor_left = 1

# Ultrasonic Sensor
address_i2c_US_sensor_right = 0xE0 >> 1
address_i2c_US_sensor_left = 0xE2 >> 1 # CHANGE ADDRESS


# Initialisation 

# IR Sensor RIGHT
IR_sensor_right = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor_right)
# IR Sensor LEFT
IR_sensor_left = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor_left)

# Ultrasonic Sensor RIGHT
US_sensor_right = SRF08(address_US_sensor_right, 2)
# Ultrasonic Sensor LEFT
US_sensor_left = SRF08(address_US_sensor_left, 2) 


# Functions

def getDistance_IRsensor():
    IR_sensor_right.getAnalogValue()
    IR_sensor_left.getAnalogValue()
    IR_sensor_right.distance # output this to ROS
    IR_sensor_left.distance # output this to ROS

def getDistance_USsensor():
    US_sensor_right.readEcho(1)
    US_sensor_left.readEcho(1)
    US_sensor_right.echo[0] # output this to ROS
    US_sensor_left.echo[0] # output this to ROS

def init_node():
    rospy.init_node("")







