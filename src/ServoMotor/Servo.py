#!/usr/bin/python

import Adafruit_PCA9685

# ===========================================================================

# Servo Class

# ===========================================================================

class Servo :

  PCA9685_PWM_RES = 4096

  # Private Fields
  freq = 50
  pulse_length_center = 1500 # in us
  coef_deg_length = float(500/90) # coeficient used to convert an angle to a pulse length (in us/deg)

  # Constructor
  def __init__(self, address=0x40, freq=50, debug=False):
	self.pwm = Adafruit_PCA9685.PCA9685(address)
	self.freq = freq # Frequency of the PWM for the servo

	self.address = address # Address of the PCA9685 controller
	self.debug = debug # Make sure the specified mode
	
  def setAngle(self, channel, angle):
	pulse_length = (self.pulse_length_center + angle*self.coef_deg_length)*self.PCA9685_PWM_RES * self.freq / 1000000
	print (pulse_length)
	self.pwm.set_pwm(channel, 0, int(pulse_length))

  def setPulseLengthCenter(self, pulse_length=1500):
	self.pulse_length_center = pulse_length

  def setCoef(self, coef=float(500/90)):
	self.coef_deg_length = float(coef)

