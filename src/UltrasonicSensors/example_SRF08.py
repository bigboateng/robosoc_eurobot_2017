#!/usr/bin/python

from SRF08 import SRF08

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the SRF08
# Change the I2C address to match your module address
address = 0xE0 # WARNING : if you use the address provided by i2cdetect,
	       #	   address = [addr] << 1
UltraSonic_sensor = SRF08(address >> 1, 2, False)
i = 1

# Code to change the address of an SRF08 module
if (0): # Put 1 to change address, 0 otherwise
	new_address = 0xE0 # default : 0xE0
	UltraSonic_sensor.changeAddress(new_address)
	i = 0
	print "Address changed for : %s" % (format(new_address, '#04x')) 

''' 
while i:
	#i = 0
	UltraSonic_sensor.readEcho(1)
	print "Echo1 : %d" % (UltraSonic_sensor.echo[0])
 
'''
while i:
	#i = 0
	UltraSonic_sensor.readEcho()
	print "Echo1 : %d" % (UltraSonic_sensor.echo[0])
	print "Echo2 : %d" % (UltraSonic_sensor.echo[1])
	print "Echo3 : %d" % (UltraSonic_sensor.echo[2])
	print "Echo4 : %d" % (UltraSonic_sensor.echo[3])
	print "Echo5 : %d" % (UltraSonic_sensor.echo[4])
	print "Echo6 : %d" % (UltraSonic_sensor.echo[5])
	print "Echo7 : %d" % (UltraSonic_sensor.echo[6])
	print "Echo8 : %d" % (UltraSonic_sensor.echo[7])
	print "Echo9 : %d" % (UltraSonic_sensor.echo[8])
	print "Echo10 : %d" % (UltraSonic_sensor.echo[9])
	print "Echo11 : %d" % (UltraSonic_sensor.echo[10])
	print "Echo12 : %d" % (UltraSonic_sensor.echo[11])
	print "Echo13 : %d" % (UltraSonic_sensor.echo[12])
	print "Echo14 : %d" % (UltraSonic_sensor.echo[13])
	print "Echo15 : %d" % (UltraSonic_sensor.echo[14])
	print "Echo16 : %d" % (UltraSonic_sensor.echo[15])
	print "Echo17 : %d" % (UltraSonic_sensor.echo[16])
	print "------------------------------------------"
#'''
