#!/usr/bin/env python

#import IRsensor
from IRsensor import IRsensor
import time

SPI_PORT   = 0
SPI_DEVICE = 0
channel_mcp_IR_sensor = 0

IR_sensor = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor)

while 1:
	IR_sensor.getAnalogValue()
	print "Distance : %d" % (IR_sensor.distance)
	time.sleep(0.1)
