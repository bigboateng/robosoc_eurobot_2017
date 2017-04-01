#!/usr/bin/env python
import rospy

from os import sys, path

sys.path.insert(0, "/MD25") #map_loader.MapLoader
from PythonAstar.map_loader import MapLoader
from PythonAstar.map_helper import MapHelper
from MD25 import MD25
import Robot
from std_msgs.msg import String
from PrimaryRobotState import * #import States, ArmState
from PID import PID
import time
from math import radians
import rospy
from nav_msgs.msg import Odometry

# object to hold robot parameters
primary_robot = Robot.Robot(axle_length=0.10265,wheel_diameter=0.097)
# control motors
motor_controller = MD25(address=0x58, robot=primary_robot)
motor_controller.resetEncoders()
motor_controller.set_acceleration(1)

# starting state = wait for start buttin
state = States.GET_PATH

# number of cylinders that robot carry
cylinders = 0

# current possition - updated by subscribing to ros
position = [0.0,0.0,0.0]

# state of robotic arm
arm_state = ArmState.COMPLETE
# refresh rate of program
refresh_rate = 80 # 50 Hz

#configure A*
my_map_loader = MapLoader(file_path = "./PythonAstar/300by200")
my_map, array = my_map_loader.load_map(0)

# Actions to be done
global_actions = [MapHelper().get("smallcrater", "blue", "B"), States.PICK_UP_CYLINDER, States.PUT_DOWN_CYLINDER, States.GET_TO_CYLINDER]
#global_path_instructions = [("drive", 0.4),("drive", 0.35) , ("bearing", -3.14)]
global_path_instructions = []

# robots current posision
currentPosX, currentPosY = 18,10

# PID set points
set_point_left = 0.0
set_point_right = 0.0

leftPID = PID()
rightPID = PID()

# publishers
arm_controller_pub = None # set it up in initialize_node function
holder_controller_pub = None # set it up in initialize_node function

def start_stop_program(msg): # this will state the main state machine
    global state
    if msg.data == "start":
        state = States.CHOOSE_TASK # begin program
        rospy.loginfo("Starting program...")
    elif msg.data == "stop": # 90 seconds, stop program
        state = States.EMERGENCY_STOP
        rospy.loginfo("Stoping program...")

def doAction(actionAsString): # find what action to do and publish
    global arm_controller_pub, state, arm_state
    if actionAsString == "grabber_arm_up":
        arm_controller_pub.pub("up")
        state = States.WAITING_FOR_ACTION_FINISH
        arm_state = ArmState.BUSY
    elif actionAsString == "grab_cylinder":
        # includes: arm_down, arm_open, drive to cylinder, arm_close 
    elif actionAsString == "grabber_arm_down":
        arm_controller_pub.pub("down")
        state = States.WAITING_FOR_ACTION_FINISH
        arm_state = ArmState.BUSY
    elif actionAsString == "store_cylinder":
        arm_controller_pub.pub("open")
        state = States.WAITING_FOR_ACTION_FINISH
        arm_state = ArmState.BUSY
    elif actionAsString == "go_to":
        # The next commands will be: object (e.g. cylinder), colour, letter
    elif actionAsString == "drive":
        # Drive a set distance which will be the next instruction, can be negative i.e. reverse
    elif actionAsString == "place_cylinder":
        # Drop the cylinder into moonbase
    elif actionAsString == "rotation_arm_down":
        # Arm to rotate the cylinder goes down (up is normal pos)
    elif actionAsString == "rotate_cylinder":
        # Check the light and rotate 1 cylinder until correct colour
    elif actionAsString == "rotation_arm_up":
        # Arm to rotate the cylinder goes up
    elif actionAsString == "armClose":
        arm_controller_pub.pub("close")
        state = States.WAITING_FOR_ACTION_FINISH
        arm_state = ArmState.BUSY
    elif actionAsString == "holderRelease":
	holder_controller_pub.pub("release")
    elif actionAsString == "holderReturn":
	holder_controller_pub.pub("start")

def action_complete(data): # call back when arm action is complete
    global arm_state, state
    arm_state == ArmState.COMPLETE


def turnLeft(angle):
    global set_point_left, set_point_right, state, leftPID, rightPID
    set_point_left = -(primary_robot.axle_length / 2) * angle  # S = R * Theta
    set_point_right = (primary_robot.axle_length / 2) * angle
    leftPID.setPoint(set_point_left)
    rightPID.setPoint(set_point_right)
    state = States.GO_TO_GOAL

def updatePos(data):
	global position
	position = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.orientation.x]
	print("Updated position to: ",position)

def initialize_node():
    global state, global_actions, arm_state, global_path_instructions, set_point_left, set_point_right, leftPID, rightPID
    global position
    end_goal_position = (95,20,0)
    rospy.init_node("primary_robot_node") # Initialize the ros node
    rate = rospy.Rate(refresh_rate) # refresh rate
    # Publishers
    arm_controller_pub = rospy.Publisher("arm_controller", String, queue_size=10)
    holder_controller_pub = rospy.Publisher("holder", String, queue_size=10)
    # Subscribers
    start_stop_program_subscriber = rospy.Subscriber("start_stop_program", String, start_stop_program)
    arm_action_complete_subscriber = rospy.Subscriber("arm_action_complete", String, action_complete)
    #Subscribing to ros localization
    rospy.Subscriber("odomtery/filtered", Odometry, updatePos)
    # left and right wheel PID
    has_reached_goal = False #0.11 * (3.14 / 2)
    left_complete = False
    right_comeplete = False
    leftPID.setPoint(set_point_left)
    rightPID.setPoint(set_point_right)
    while not rospy.is_shutdown():
        #motor_controller.updatePosition2()
        #print("X = {}, Y = {}".format(motor_controller.getXPosition(), motor_controller.getYPosition()))
        if state == States.WAIT_FOR_START_BUTTON:
            # do nothing basically, waiting for someone to pull start button
            print("Waiting to start")
            state = States.GO_TO_GOAL
        elif state == States.CHOOSE_TASK:
            if not len(global_path_instructions) == 0: # We need to move
                move_action = global_path_instructions[0]
                if move_action[0] == "drive":
                    print("straight")
                    motor_controller.resetEncoders()
                    set_point_left = move_action[1]/100
                    set_point_right = move_action[1]/100
                    leftPID.setPoint(set_point_left)
                    rightPID.setPoint(set_point_right)
                    left_complete = False
                    right_comeplete = False
                    print("SETPOINT LEFT= {}, SET_POINT_RIGHT={}, current={},{}".format(set_point_left,set_point_right, motor_controller.get_left_distance(), motor_controller.get_right_distance()))
                    state = States.GO_TO_GOAL
                elif move_action[0] == "bearing":
                    print("TURNING LEFT")
                    motor_controller.resetEncoders()
                    angle = move_action[1]
                    set_point_left = -(primary_robot.axle_length / 2) * radians(angle)  # S = R * Theta
                    set_point_right = (primary_robot.axle_length / 2) * radians(angle)
                    leftPID.setPoint(set_point_left)
                    rightPID.setPoint(set_point_right)
                    left_complete = False
                    right_comeplete = False
                    state = States.GO_TO_GOAL
                    print("SETPOINT LEFT= {}, SET_POINT_RIGHT={}, current={},{}".format(set_point_left,set_point_right, motor_controller.get_left_distance(), motor_controller.get_right_distance()))
            else:
                if not len(global_actions) == 0: # then are actions to do
                    action  = global_actions[0]
                    if type(action) == 'tuple' and len(action) == 3: # this is a target goal
                        end_goal_position = action[0]
                        state = States.GET_PATH
                    elif type(action) == "str": # then its an action like pick up cylinder
                        state = action
                else:
					print("no instructions to do")
        elif state == States.GET_PATH:
            print("State = GET PATH")
            end_position = global_actions[0]
            print( (end_position[0],end_position[1]))
            print(end_position[2])
            global_path_instructions = my_map.get_instructions((position[0], position[1]), (end_position[0],end_position[1]), end_position[2])
            print(global_path_instructions)
            state = States.GO_TO_GOAL
        elif state == States.WAITING_FOR_ACTION_FINISH:
            # waiting for something like grabber to finish picking up
            if arm_state == ArmState.BUSY:
                pass # do nothing
            elif arm_state == ArmState.COMPLETE: # has finished picking or dropping something
                state == States.CHOOSE_TASK
        elif state == States.GO_TO_GOAL: # use PID to drive to target distance or turning
            #print("Left Dist = {}, right Dist={}, Left error={}, right error={}".format(motor_controller.get_left_distance(), motor_controller.get_right_distance(), leftPID.getError(), rightPID.getError()))
            left_speed = leftPID.update(motor_controller.get_left_distance())
            right_speed = rightPID.update(motor_controller.get_right_distance())
            if abs(leftPID.getError()) > 0.01:
                motor_controller.set_left_speed(225 - left_speed)
                #motor_controller.forward(255-left_speed)
            else:
                left_complete = True

            if abs(rightPID.getError()) > 0.01:
                motor_controller.set_right_speed(225 - right_speed)
            else:
                right_comeplete = True 
            if left_complete: #and right_comeplete:
                state = States.STOP_MOTORS
        elif state == States.STOP_MOTORS:  # stop the motors
            motor_controller.stop()
            motor_controller.set_right_speed(128)
            motor_controller.set_left_speed(128)
            time.sleep(1)
            print("STOP MOTOR ")
            state = States.CHOOSE_TASK

        elif state == States.EMERGENCY_STOP:
            motor_controller.stop()
            # stop drum here
        elif state == States.GET_TO_CYLINDER:
        	print("getting to cylinder")
       		doAction("armClose")
        	doAction("armDown")
	    	doAction("armOpen")
        	global_path_instructions = [("drive", 0.05)]
        elif state == States.PICK_UP_CYLINDER:
			print("picking up cylinder")
	    	doAction("armClose")
	    	doAction("armUp")
	    	if cylinders <4:
			doAction("armOpen")
	    	cylinders+= 1
            global_path_instructions(("drive", -0.05))
		elif state == States.PUT_DOWN_CYLINDER:
			print("putting down cylinder")
		    doAction("holderRelease")
		    doAction("holderReturn")
		    cylinders-= 1
            if(cylinders>2):
                doAction("armOpen")

        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
