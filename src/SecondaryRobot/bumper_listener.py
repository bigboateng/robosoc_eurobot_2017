"""
This node listens for button clicks and alert the main program
"""
import rospy
from std_msgs.msg import String
from os import sys, path
sys.path.append('../LimitSwitch')
from Bumper import Bumper
