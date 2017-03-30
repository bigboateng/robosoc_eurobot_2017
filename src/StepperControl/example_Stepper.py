import Stepper
import time

# Define GPIO signals to use
# Physical pins 11,15,16,18
# GPIO17,GPIO22,GPIO23,GPIO24
StepPins1 = [17,22,23,24]
StepPins2 = [5,6,13,19]

steps = 4076/2
revo = 1/2

Stepper.setupStepper(1, StepPins1, 4076)
Stepper.setupStepper(2, StepPins2, 4076)
Stepper.setSpeed(1200) # 1200 steps/sec
Stepper.moveBoth_revo(revo)
Stepper.setDirBoth(-1)
time.sleep(0.5)
Stepper.moveBoth_step(steps)
time.sleep(0.5)
Stepper.move_step(1, 1000)
time.sleep(0.5)
Stepper.setDir(2, 1)
Stepper.move_step(2, 1000)


