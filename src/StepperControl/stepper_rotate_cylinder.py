import Stepper
import time

# Pins for the two stepper motors
stepper = 1
stepperPins1 = [4,17,27,22] # Right stepper
totalSteps = 4076
speed = 1200 # 1200 steps/sec max @5V

revo = 3

Stepper.setupStepper(stepper, stepperPins1, totalSteps)
Stepper.setSpeed(speed) 
Stepper.setDir(stepper,1)

Stepper.move_revo(stepper, revo)



