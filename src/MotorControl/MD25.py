#!/usr/bin/python

import time
from Adafruit_I2C import Adafruit_I2C
from Robot import Robot
from math import cos, sin
# ===========================================================================
# MD25 Class
# ===========================================================================
class MD25 :
  i2c = None

  # Operating Modes
  __MD25_STANDARD                 = 0
  __MD25_NEGATIVE_SPEEDS          = 1
  __MD25_SPEED_AND_TURN           = 2
  __MD25_NEGATIVE_SPEEDS_AND_TURN = 3

  # MD25 Registers
  __MD25_SPEED_1 = 0
  __MD25_SPEED_2 = 1
  __MD25_ENC_1A = 2
  __MD25_ENC_1B = 3
  __MD25_ENC_1C = 4
  __MD25_ENC_1D = 5
  __MD25_ENC_2A = 6
  __MD25_ENC_2B = 7
  __MD25_ENC_2C = 8
  __MD25_ENC_2D = 9
  __MD25_BATTERY_VOLTAGE = 10
  __MD25_MOTOR_1_CURRENT = 11
  __MD25_MOTOR_2_CURRENT = 12
  __MD25_SOFTWARE_VERSION = 13
  __MD25_ACCELERATION = 14
  __MD25_MODE = 15
  __MD25_COMMAND = 16

  __MD25_RESET_ENCODER_REGISTERS = 0x20
  __MD25_DISABLE_AUTO_SPEED_REGULATION = 0x30
  __MD25_ENABLE_AUTO_SPEED_REGULATION = 0x31
  __MD25_DISABLE_2_SECOND_TIMEOUT = 0x32
  __MD25_ENABLE_2_SECOND_TIMEOUT = 0x33
  __MD25_CHANGE_I2C_ADDRESS_1 = 0xA0
  __MD25_CHANGE_I2C_ADDRESS_2 = 0xAA
  __MD25_CHANGE_I2C_ADDRESS_3 = 0xA5


  # Private Fields
  _enc_1a = 0
  _enc_1b = 0
  _enc_1c = 0
  _enc_1d = 0
  _enc_2a = 0
  _enc_2b = 0
  _enc_2c = 0
  _enc_2d = 0
  _battery_voltage = 0
  _motor_1_current = 0
  _motor_2_current = 0
  _software_revision = 0

  # Pi
  PI = 3.14159

  # wheels pulses per revolustion
  _PULSES_PER_REVOLUTION = 360

  # Constructor
  def __init__(self, address=0x58, mode=1, debug=False, robot=None):
    self.i2c = Adafruit_I2C(address)
    self.address = address
    self.debug = debug # Make sure the specified mode is in the appropriate range 
    if ((mode < 0) | (mode > 3)): 
      if (self.debug): 
        print "Invalid Mode: Using STANDARD by default"
      self.mode = self.__MD25_STANDARD
    else:
      self.mode = mode
    #self.readBatteryVoltage()
    #self.readData()
    self.PI = 3.14159
    self.robot = robot
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.theta = 0.0
    self.PULSES_PER_REVOLUTION = 360
    self.mul_count = self.PI * self.robot.wheel_diameter / self.PULSES_PER_REVOLUTION

  def forward(self, speed=100):
    self.i2c.write8(self.__MD25_SPEED_1, speed)
    self.i2c.write8(self.__MD25_SPEED_2, speed)

  def stop(self):
    self.i2c.write8(self.__MD25_SPEED_1, 128)
    self.i2c.write8(self.__MD25_SPEED_2, 128)

  def turn(self, speed1=10, speed2=1):
    self.i2c.write8(self.__MD25_SPEED_1, speed1)
    self.i2c.write8(self.__MD25_SPEED_2, speed2)

  def showBatteryVoltage(self):
    "Reads the battery voltage"
    print "DBG: BATTERY_VOLTAGE = %6d" % (self._battery_voltage)

  def readData(self):
    "Reads the data from the MD25" 
    self._enc_1a = self.i2c.readS8(self.__MD25_ENC_1A) 
    if (self.debug):
      self.showData()

  def showData(self):
      "Displays the calibration values for debugging purposes"
      print "DBG: ENC_1A = %6d" % (self._enc_1a)

  def readBatteryVoltage(self):
      "Reads the battery voltage from md25"
      self._battery_voltage = self.i2c.readS8(6)
      print "Voltage = %s" %str(self._battery_voltage/10.0)
	
  def resetEncoders(self):
    "Resets the encoder values"
    self.i2c.write8(self.__MD25_COMMAND, self.__MD25_RESET_ENCODER_REGISTERS)

  def readEncoder(self, type=1):
      "Reads the encoder 1 value"
      if type == 1:
        base_reg = 2
      else:
        base_reg = 6
      b3 = self.i2c.readS8(base_reg)
      b2 = self.i2c.readU8(base_reg+1)
      b1 = self.i2c.readU8(base_reg+2)
      b0 = self.i2c.readU8(base_reg+3)
      encoder = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0	
      return encoder

  def getEncoderValues(self):
    "Returns the both encoder values"
    encoder1 = readEncoder(1)
    encoder2 = readEncoder(2)
    return encoder1, encoder2


  def updatePosition(self):
    # Updates x, y, theta
    PI = self.PI
    left_ticks = self.readEncoder(1)
    right_ticks = self.readEncoder(2)
    dist_left = left_ticks * self.mul_count
    dist_right = right_ticks * self.mul_count
    cos_current = cos(self.theta)
    sin_current = sin(self.theta)

    if abs(left_ticks -right_ticks) < 1e-6: # basically going straight
      # moving in a straight line
      self.pos_x += dist_left * cos_current
      self.pos_y += dist_left * sin_current
    else:
      # moving in an arc
      expr1 = self.robot.axle_length * (dist_right + dist_left) / (2.0 * (dist_right - dist_left))
      right_minus_left = dist_right - dist_left
      self.pos_x += expr1 * (sin(right_minus_left / self.robot.axle_length + self.theta)-sin_current)
      self.pos_y -= expr1 * (cos(right_minus_left/self.robot.axle_length + self.theta) - cos_current)
    
      # new orientation
      self.theta = right_minus_left / self.robot.axle_length

      # Keep in range of -Pi to +pi
      while self.theta > 2*PI:
        self.theta -= (2.0*PI)
      while self.theta < -2*PI:
        self.theta += (2.0*PI)

  def getXPosition(self):
    return self.pos_x

  def getYPosition(self):
    return self.pos_y

  def getTheta(self):
    return self.theta






  
