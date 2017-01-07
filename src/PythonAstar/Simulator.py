#!/usr/bin/env python
import threading
import time
from __future__ import print_function
from math 
from random import randint

gameLength = 90 # sec
exitFlag = 0 
obstacleFlag = 0 

class LookForObstacleThread(threading.Thread):

	"""
		name - name of the thread
		lifetime - lifetime of the thread is length of the game in seconds
		rate - the number of checks for obstacles per seconds
	"""
    def __init__(self, name, lifetime, rate): 
    	threading.Thread.__init__(self)
        self.name = name
        self.lifetime = lifetime
        self.rate = rate
        self.counter = lifetime * rate


    # perform checks for obstacles and change the flag when an obstacle is found, update the position of the obstacle
    # if the obstacle is not found then change the flag to 0 again
    @synchronized
    def checkObstacleSensors(self):
    	obstacleFlag = math.round(random.random())
    	obstacleDetails = (1, 2) # position of the obstacle
    	continue

    def run(self):
    	while (self.counter):
    		if exitFlag:
            	threadName.exit();
            checkObstacleSensors()
    		time.sleep(1/rate)
    		self.counter -= 1

# Create new threads
lookForObstacleThread = LookForObstacleThread("ListeningThread", gameLength, 0.5)

# Start new Threads
lookForObstacleThread.start()