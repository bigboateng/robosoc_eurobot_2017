#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from os import sys, path
from libs import Servo
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


action_complete_pub = None

def send_action_complete():
    global action_complete_pub
    action_complete_pub.publish("done")
    rospy.loginfo("action done")


#TODO check delays
def grabberOpen():
    print "Open grabber (-90 deg.)"
    grabber_servo.setAngle(-90)
    time.sleep(0.2)
    send_action_complete()

def grabberClose():
    rospy.loginfo("close arm here")
    print "Close grabber (40 deg.)"
    grabber_servo.setAngle(70)
    time.sleep(0.7)
    send_action_complete()
    #grabber_servo.setAngle(0) # stop giving current

#TODO finish arm functions with actual python code
def armUp():
    rospy.loginfo("go up with arm here")
    grabberClose()
    arm_servo.setAngle(7)
    time.sleep(0.8)
    send_action_complete()

def armDown():
    rospy.loginfo("go down with arm here")
    grabberClose()
    arm_servo.setAngle(-88)
    time.sleep(0.5)
    send_action_complete()

def defaultPosition():
    armUp()
    grabber_servo.setAngle(0)
    time.sleep(1)
    send_action_complete()

def armControl(data):
    rospy.loginfo("command received")
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
    send_action_complete()

def holderStart():
    rospy.loginfo("return holder to starting position here")
    send_action_complete()

def holderControl(data):
    if(data.data=="release"):
        holderRelease()
    elif(data.data=="start"):
        holderStart()
    else:
        rospy.loginfo("wrong message")



def listener():
    global action_complete_pub
    rospy.init_node('servo_subscriber', anonymous=True)

    # Publisher for when action is complete 
    action_complete_pub = rospy.Publisher("arm_action_complete", String, queue_size=10)

    rospy.Subscriber("arm_controller", String, armControl)

    rospy.Subscriber("holder", String, holderControl)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except ROSInterruptException:
        pass
