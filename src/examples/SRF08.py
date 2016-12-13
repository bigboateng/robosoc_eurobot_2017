#!/usr/bin/python

import time
from Adafruit_I2C import Adafruit_I2C

# ===========================================================================
# SRF08 Class
# ===========================================================================

class SRF08 :
  i2c = None

  # SRF08 Registers
  # Write
  SRF08_COMMAND = 0
  SRF08_MAX_GAIN = 1
  SRF08_RANGE = 2
  # Read
  SRF08_SOFTWARE_VERSION = 0
  SRF08_LIGHT_SENSOR = 1
  SRF08_ECHO_1_MSB = 2
  # Read-only
  SRF08_ECHO_1_LSB = 3
  SRF08_ECHO_2_MSB = 4
  SRF08_ECHO_2_LSB = 5
  SRF08_ECHO_3_MSB = 6
  SRF08_ECHO_4_LSB = 7
  SRF08_ECHO_4_MSB = 8
  SRF08_ECHO_5_LSB = 9
  SRF08_ECHO_5_MSB = 10
  SRF08_ECHO_6_LSB = 11
  SRF08_ECHO_6_MSB = 12
  SRF08_ECHO_7_LSB = 13
  SRF08_ECHO_7_MSB = 14
  SRF08_ECHO_8_LSB = 15
  SRF08_ECHO_8_MSB = 16
  SRF08_ECHO_9_LSB = 17
  SRF08_ECHO_9_MSB = 18
  SRF08_ECHO_10_LSB = 19
  SRF08_ECHO_10_MSB = 20
  SRF08_ECHO_11_LSB = 21
  SRF08_ECHO_11_MSB = 22
  SRF08_ECHO_12_LSB = 23
  SRF08_ECHO_12_MSB = 24
  SRF08_ECHO_13_LSB = 25
  SRF08_ECHO_13_MSB = 26
  SRF08_ECHO_14_LSB = 27
  SRF08_ECHO_14_MSB = 28
  SRF08_ECHO_15_LSB = 29
  SRF08_ECHO_15_MSB = 30
  SRF08_ECHO_16_LSB = 31
  SRF08_ECHO_16_MSB = 32
  SRF08_ECHO_17_LSB = 34
  SRF08_ECHO_17_MSB = 35

  # SRF08 Commands
  SRF08_RANGING_MODE_RESULT_INCHES = 0x50
  SRF08_RANGING_MODE_RESULT_CENTIMETERS = 0x51
  SRF08_RANGING_MODE_RESULT_MICROSECONDS = 0x52
  SRF08_ANN_MODE_RESULT_INCHES = 0x53
  SRF08_ANN_MODE_RESULT_CENTIMETERS = 0x54
  SRF08_ANN_MODE_RESULT_MICROSECONDS = 0x55  
  SRF08_CHANGE_I2C_ADDRESS_1 = 0xA0
  SRF08_CHANGE_I2C_ADDRESS_2 = 0xA5
  SRF08_CHANGE_I2C_ADDRESS_3 = 0xAA

  # Private Fields
  range_mm # Range in mm (min 43mm, max 11008mm)
  mode # 1: INCHES / 2: CENTIMETERS / 3: MICROSECONDS 
  echo[17]

  # Constructor
  def __init__(self, address=0xE0, mode=2, debug=False):
    self.i2c = Adafruit_I2C(address)

    self.address = address
    self.debug = debug # Make sure the specified mode is in the appropriate range 
    
	self.mode = mode
	self.range_mm = 11008
	self.i2c.write8(self.SRF08_RANGE, ((self.range_mm-43)/43) )
	
#  def writeRange_mm(self, range)
#	self.i2c.write8(self.SRF08_RANGE, ((range-43)/43) )
	
  def setMode(self, mode)
	self.mode = mode
	
  def readEcho(seft) #, echo=1)
	if (self.mode = 1)
		self.i2c.write8(SRF08_COMMAND, SRF08_RANGING_MODE_RESULT_INCHES)
		time.sleep(self.range_mm * 0.065/11008)
		i = 0
		while i < 17
			echo[i] = self.i2c.readU8(self.SRF08_ECHO_1_LSB+i) + (self.i2c.readU8(self.SRF08_ECHO_1_MSB+i) * 255)
			i++;
	else if (self.mode = 2)
		self.i2c.write8(SRF08_COMMAND, SRF08_RANGING_MODE_RESULT_CENTIMETERS)
		time.sleep(self.range_mm * 0.065/11008)
		i = 0 
		while i < 17
			echo[i] = self.i2c.readU8(self.SRF08_ECHO_1_LSB+i) + (self.i2c.readU8(self.SRF08_ECHO_1_MSB+i) * 255)
			i++
	else if (self.mode = 3)
		self.i2c.write8(SRF08_COMMAND, SRF08_RANGING_MODE_RESULT_MICROSECONDS)
		time.sleep(self.range_mm * 0.065/11008)
		i = 0 
		while i < 17
			echo[i] = self.i2c.readU8(self.SRF08_ECHO_1_LSB+i) + (self.i2c.readU8(self.SRF08_ECHO_1_MSB+i) * 255)
			i++
	else
		i = 0 
		while i < 17
			echo[i] = 0
			i++;
			
		print "Undefined mode number"
			