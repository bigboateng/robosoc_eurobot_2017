#!/usr/bin/env python
from master_robot.srv import *
import rospy

def handle_add_two_int(req):
    print("Returning [{} + {} = {}]".format(req.a, req.b, (req.a+req.b)))
    return AddTwoIntsResponse(req.a+req.b)
    

def add_two_ints_server():
    rospy.init_node("add_two_ints_server")
    service = rospy.Service("add_two_ints", AddTwoInts, handle_add_two_ints)
    print("Ready to handle AddTwoInts")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
