from MD25 import MD25
from Robot import Robot
import time

secondary_robot = Robot(0.10265, 0.97)
controller = MD25.MD25(0x59,1,True,secondary_robot)
controller.resetEncoders()

def driveForward(distance):
	for x in range(distance):
		controller.forward(100)
		controller.updatePosition()
		time.sleep(0.1)
		print("X = {}, Y = {}".format(controller.getXPosition(), controller.getYPosition()))

if __name__ == '__main__':
	driveForward(5)