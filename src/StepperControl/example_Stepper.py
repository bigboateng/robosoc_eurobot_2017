#!/usr/bin/python

import threading
import time
from Stepper import Stepper

# ===========================================================================
# Example code 
# ===========================================================================

speed = 1200 # steps/second, max: 1200


# Initialisation 
# Stepper(stepPins=[17,22,23,24], totalSteps=4076, diameter_mm=5)
stepper = Stepper([17,22,23,24], 4076, 5)
stepper.setSpeed(speed)
stepper.move_steps(4000)
'''
thread = threading.Thread(target=stepper.move_step, args=(1000))
thread.daemon = True
thread.start()
'''
#stepper.move_step(4076)
#stepper.run()
print "Delay"
time.sleep(10)
print "End"

