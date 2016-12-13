#!/usr/bin/python

from SRF08 import SRF08

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the SRF08
UltraSonic_sensor = SRF08(0xE0, 2, False)
UltraSonic_sensor.readEcho();
print "Echo1 : %d" % (UltraSonic_sensor.echo[0])
print "Echo2 : %d" % (UltraSonic_sensor.echo[1])
print "Echo3 : %d" % (UltraSonic_sensor.echo[2])