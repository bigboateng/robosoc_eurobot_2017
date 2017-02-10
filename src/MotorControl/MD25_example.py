##!/usr/bin/python

from MD25 import MD25
import time
from Robot import Robot
# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the MD25 and use STANDARD mode (default value)controller.resetEncoders()
robot = Robot(0.181, 0.1)
controller = MD25(0x58,1,True,robot)
#controller.resetEncoders()
controller.forward()
time.sleep(2)
controller.stop()
#controller.updatePosition()
#controller.forward()
#time.sleep(2)
#controller.stop()
#controller.updatePosition()

#print("X = {}, Y = {}, Theta = {}".format(controller.getXPosition(), controller.getYPosition(), controller.getTheta()))
#controller.turn()
#time.sleep(1)
#controller.stop()
#controller.updatePosition()
#time.sleep(1)
#controller.forward()
#time.sleep(1.0)
#controller.stop()
#controller.turn()
#time.sleep(1.0)
#controller.stop()
#controller.updatePosition()
#print("X = {}, Y = {}, Theta = {}".format(controller.getXPosition(), controller.getYPosition(), controller.getTheta()))
#controller.readEncoder1()
