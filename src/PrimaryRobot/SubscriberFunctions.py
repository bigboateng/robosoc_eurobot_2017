#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from os import sys, path
sys.path.append('../ServoMotor')
import Servo
import time

# main servo (HX12K)
arm_servo = Servo.Servo(0x40, 0, 50)
arm_servo.setPulseLengthMin(550)
arm_servo.setPulseLengthMax(2450)
arm_servo.setMaxAngle(180)
# Secondary servo (SM-S2309S)
grabber_servo = Servo.Servo(0x40, 1, 50)
grabber_servo.setPulseLengthMin(550)
grabber_servo.setPulseLengthMax(2450)
grabber_servo.setMaxAngle(180)
# Release Servo



#TODO check delays
def grabberOpen():
    print "Open grabber (-90 deg.)"
    grabber_servo.setAngle(-90)

def grabberClose():
    rospy.loginfo("close arm here")
    print "Close grabber (40 deg.)"
    grabber_servo.setAngle(40)
    time.sleep(0.1)
    #grabber_servo.setAngle(0) # stop giving current

#TODO finish arm functions with actual python code
def armUp():
    rospy.loginfo("go up with arm here")
    grabberClose()
    time.sleep(0.2)
    arm_servo.setAngle(7)
    time.sleep(0.1)

def armDown():
    rospy.loginfo("go down with arm here")
    grabberClose()
    time.sleep(0.1)
    arm_servo.setAngle(-88)

def defaultPosition():
    armUp()
    time.sleep(0.3)
    grabber_servo.setAngle(0)

def armControl(data):
    if(data.data=="up"):
        armUp()
    elif(data.data=="down"):
        armDown()
    elif(data.data=="close"):
        grabberClose()
    elif(data.data=="open"):
        grabberOpen()
    elif(data.data=="default"):
        defaultPosition()
    else:
        rospy.loginfo("wrong message")
    
#TODO finish holder functions with actual python code
def holderRelease():
    rospy.loginfo("release cylinder here")

def holderStart():
    rospy.loginfo("return holder to starting position here")

def holderControl(data):
    if(data.data=="release"):
        holderRelease()
    elif(data.data=="start"):
        holderStart()
    else:
        rospy.loginfo("wrong message")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('servo_subscriber', anonymous=True)

    rospy.Subscriber("arm_controller", String, armControl)

    rospy.Subscriber("holder", String, holderControl)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__' and __package__ is None:
    try:
        listener()
    except ROSInterruptException:
        pass
