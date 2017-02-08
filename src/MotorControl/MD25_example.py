#!/usr/bin/python

from MD25 import MD25
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the MD25 and use STANDARD mode (default value)controller.resetEncoders()
controller = MD25(0x58,1,True)
controller.resetEncoders()
controller.forward()
time.sleep(1)
controller.stop()
#controller.turn()
#time.sleep(1)
#controller.stop()
time.sleep(1)
controller.forward()
time.sleep(1.0)
controller.stop()
#controller.turn()
#time.sleep(1.0)
controller.stop()
#controller.readEncoder1()
