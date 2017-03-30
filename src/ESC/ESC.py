#!/usr/bin/python

import Adafruit_PCA9685

# ===========================================================================

# ESC Class

# ===========================================================================

class ESC :

  PCA9685_PWM_RES = 4096

  # Private Fields
  pulse_length_min = 1000 # in us
  pulse_length_max = 2000 # in us

  # Constructor
  def __init__(self, address=0x40, channel=0, freq=50, debug=False):
	self.pwm = Adafruit_PCA9685.PCA9685(address)
	self.address = address # Address of the PCA9685 controller
	self.channel = channel # Channel of the PWM for the ESC	
	self.freq = freq # Frequency of the PWM for the ESC
	self.pwm.set_pwm_freq(freq)
	
	if (channel < 0 | channel > 15) :
		print "Error: The channel number must be between 0 and 15"
 
	self.debug = debug 

  def setSpeed(self, speed): # speed: 0-100%
	if (speed >= 0 & speed <= 100) :
		pulse_length = (self.pulse_length_min + (self.pulse_length_max-self.pulse_length_min)*speed/100)*self.PCA9685_PWM_RES * self.freq / 1000000
		self.pwm.set_pwm(self.channel, 0, int(pulse_length))
	else :
		print "Error: The speed parameter must be between 0 and 100%"

  def setPulseLengthMin(self, pulse_length=1000):
	self.pulse_length_min = pulse_length

  def setPulseLengthMax(self, pulse_length=2000):
	self.pulse_length_max = pulse_length
