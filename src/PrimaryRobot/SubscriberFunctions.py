#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#TODO finish arm functions with actual python code
#Options for arm - python has no switch statement
armOptions = {"up" : armUp,
           "down" : armDown,
           "open" : armClose,
           "close" : armOpen
}

def armUp():
    print "go up with arm here"

def armDown():
    print "go down with arm here"

def armClose():
    print "open arm here"

def armOpen():
    print "close arm here"

def armControl(data):
    armOptions[data]()
    
#TODO finish holder functions with actual python code
releaseOptions = {"release" : zero,
           "start" : sqr
}

def holderRelease():
    print "release cylinder here"

def holderStart():
    print "return holder to starting position here"

def holderControl(data):

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
