from MD25 import MD25
from Robot import Robot
import time

secondary_robot = Robot(0.181, 0.1)
controller = MD25(0x58,1,True,secondary_robot)
controller.resetEncoders()

def driveForward(distance):
	for x in range(distance):
		controller.forward(100)
		time.sleep(0.1)
	controller.stop()

def turnLeft(left, angle):
	if left:
		 controller.turn(10,100)
	else:
		controller.turn(100,10)
	time.sleep(angle*0.1)
	controller.stop()

if __name__ == '__main__' and __package__ is None:
	driveForward(5)
	turnLeft(True, 10)
	driveForward(5)
