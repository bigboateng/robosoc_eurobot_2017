import Stepper
import time
import sys

# Pins for the two stepper motors
rightStepper = 1
leftStepper = 2
stepperPins1 = [17,22,23,24] # Right stepper
stepperPins2 = [5,6,13,19] # Left stepper
totalSteps = 4076
speed = 1200 # 1200 steps/sec max @5V

revo_lift_1 = 0.3
revo_lift_2 = 0.7
revo_tilt_1 = 0.4
revo_tilt_2 = 0.3
side = 'yellow' # blue side / 2: yellow side

if(side=='blue'):
	motor_tilt = rightStepper # Right stepper
elif(side=='yellow'):
	motor_tilt = leftStepper # Left stepper
else:
	sys.exit("Error: The side parameter must be 'blue' or 'yellow'")

Stepper.setupStepper(rightStepper, stepperPins1, totalSteps)
Stepper.setupStepper(leftStepper, stepperPins2, totalSteps)
Stepper.setSpeed(speed) 
Stepper.setDir(rightStepper,-1)

Stepper.moveBoth_revo(revo_lift_1)
Stepper.move_revo(motor_tilt, revo_tilt_1)
Stepper.moveBoth_revo(revo_lift_2)
Stepper.move_revo(motor_tilt, revo_tilt_2)
time.sleep(2)
Stepper.reverseDirBoth()
Stepper.moveBoth_revo(revo_lift_1)
Stepper.moveBoth_revo(revo_lift_2)
Stepper.move_revo(motor_tilt, revo_tilt_1)
Stepper.move_revo(motor_tilt, revo_tilt_2)



