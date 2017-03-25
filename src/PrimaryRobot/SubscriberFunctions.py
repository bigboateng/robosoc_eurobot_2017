#!/usr/bin/env python
import rospy
from std_msgs.msg import String

#TODO finish arm functions with actual python code
def armUp():
    rospy.loginfo("go up with arm here")

def armDown():
    rospy.loginfo("go down with arm here")

def armClose():
    rospy.loginfo("open arm here")

def armOpen():
    rospy.loginfo("close arm here")

def armControl(data):
    if(data.data=="up"):
        armUp()
    elif(data.data=="down"):
        armDown()
    elif(data.data=="close"):
        armClose()
    elif(data.data=="open"):
        armOpen()
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

    rospy.Subscriber("arm", String, armControl)

    rospy.Subscriber("holder", String, holderControl)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
