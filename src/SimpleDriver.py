from MD25 import MD25

controller = MD25(0x58,1,True,secondary_robot)
controller.resetEncoders()

def driveForvard(distance):
	for x in range distance:
		controller.forward(100)
		time.sleep(0.1)
	controller.stop()

def turnLeft(left, angle):
	if left:
		 controller.turn(1,angle)
	else:
		controller.turn(angle,1)
	controller.stop()
