#!/usr/bin/env python
import rospy

from os import sys, path

sys.path.insert(0, "/MD25") #map_loader.MapLoader
#from PythonAstar.map_loader import MapLoader
#from PythonAstar.map_helper import MapHelper
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
primary_robot = Robot.Robot(axle_length=0.181,wheel_diameter=0.097)
# control motors
motor_controller = MD25(address=0x59, robot=primary_robot)
motor_controller.resetEncoders()
motor_controller.set_acceleration(1)

# starting state = wait for start button
#state = States.WAIT_FOR_START_BUTTON
state = States.CHOOSE_TASK
#state = States.CHOOSE_TASK
prev_state = state


# error 
errorL = 0.0
errorR = 0.0
# number of cylinders that robot carry
cylinders = 0

# current position - updated by subscribing to ros
position = [0.0,0.0,0.0]

# state of robotic arm
arm_state = ArmState.COMPLETE
# refresh rate of program
refresh_rate = 40 # 50 Hz

#configure A*
my_map_loader = None #MapLoader(file_path = "./PythonAstar/300by200")
my_map, array = None, None#my_map_loader.load_map(0)

# Actions to be done
#global_actions = [["action", "del"], ["action", "grabber_arm_close"], ["action", "grabber_arm_down"], ["action", "grabber_arm_open"]]#, ["action", "grabber_arm_close"], ["action", "grabber_arm_up"], ["action", "grabber_arm_open"]]
#global_actions =  [[],['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up']]#global_path_instructions = [("drive", 0.4),("drive", 0.35) , ("bearing", -3.14)]
#global_actions = [[], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_default'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_default'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up'], ['action', 'grabber_arm_open_fully'], ['action', 'grabber_arm_default'], ['action', 'grabber_arm_down'], ['action', 'grabber_arm_open'], ['action', 'slowly_forward'], ['action', 'grabber_arm_close'], ['action', 'slowly_backward'], ['action', 'grabber_arm_up']]
#global_actions = [[],["action", "slowly_backward"]]
#global_path_instructions = [['drive', 0.9], ['rotate', 90], ['drive', 44], ['rotate', 90]]
#global_actions=[]
#global_path_instructions = [ ['rotate', 90] ,['drive', 0.19], ['rotate', 90], ['rotate', 180], ['drive', 0.82], ['rotate', -45]]
 #['drive', 0.35], ['rotate', 90], ['drive', 0.22], ['drive', -0.22], ['drive', 0.22], ['drive', -0.22],['drive', 0.22], ['drive', -0.22], ['drive', 0.22], ['drive', -0.22], ['drive', 0.22], ['drive', -0.22]
# #global_path_instructions=[
# robots current position
#global_actions = [[], ['action', 'grabber_arm_close'],['action', 'holder_release'], ['action', 'slowly_backward'], ['action', 'holder_default'],['drive', 0.1], ['action', 'holder_release'], ['action', 'slowly_backward'], ['action', 'holder_default'], ['action', 'grabber_arm_close']]
global_path_instructions = []#[['rotate', 90]]
global_actions = [[], ['drive', 0.19], ['rotate', 90],['drive', 0.19], ['rotate', 90],['drive', 0.19], ['rotate', 90]]
currentPosX, currentPosY = 18,10

# PID set points
set_point_left = 0.0
set_point_right = 0.0 

# PID controllers for the left and right motors
leftPID = PID()
rightPID = PID()

# publishers
arm_controller_pub = rospy.Publisher("arm_controller", String, queue_size=10)
holder_controller_pub = None # set it up in initialize_node function
lcd_pub = None



correct_speed = 7

def start_stop_program(msg): # this will state the main state machine
    global state
    if msg.data == "start":
        state = States.CHOOSE_TASK # begin program
        rospy.loginfo("Starting program...")
    elif msg.data == "stop": # 90 seconds, stop program
        state = States.EMERGENCY_STOP
        rospy.loginfo("Stoping program...")


def forward_correct():
    global set_point_left, set_point_right, motor_controller, errorL, errorR, state, correct_speed
    left_complete = False 
    right_complete = False
    #overshoot:
    if errorL < 0: 
        print("Left Overshoot") 
        if motor_controller.get_left_distance() > errorL:
            motor_controller.set_left_speed(128+correct_speed)
        else:
            motor_controller.set_acceleration(6)
            motor_controller.set_left_speed(128)
            left_complete = True

    if errorR < 0: 
        print("Right Overshoot")  
        if motor_controller.get_right_distance() > errorR:
            print("Right DIST={}".format(motor_controller.get_right_distance()))
            motor_controller.set_right_speed(128+correct_speed)
        else:
            motor_controller.set_acceleration(6)
            motor_controller.set_right_speed(128)
            right_complete = True

    #undershoot if forward, overshoot if backwards
    if errorL > 0:
        if motor_controller.get_left_distance() < errorL:
            motor_controller.set_left_speed(128-correct_speed)
        else:
            motor_controller.set_acceleration(6)
            motor_controller.set_right_speed(128)
            left_complete = True

    if errorR > 0:
        print("error backward entered")
        if abs(motor_controller.get_right_distance()) < abs(errorR):
            motor_controller.set_right_speed(128-correct_speed)
        else:
            motor_controller.set_acceleration(6)
            motor_controller.set_right_speed(128)
            right_complete = True
    
    if left_complete and right_complete:
        # errorOferrorleft = abs(motor_controller.get_left_distance() - errorL)
        # errorOferrorRight = abs(motor_controller.get_right_distance() - errorR)
        # if errorOferrorleft > 0.001 or errorOferrorRight > 0.001:
        #     errorL = errorOferrorleft
        #     errorR = errorOferrorRight
        #     correct_speed = 4

        #     motor_controller.resetEncoders()

        state = States.STOP_MOTORS
    # else:
    #     print("errorOferrorleft={}, errorOferrorRight={}".format(errorOferrorleft, errorOferrorRight))
    #     state = States.STOP_MOTORS



def turn_left_correct():
    pass

def turn_right_correct():
    pass

def turn(bearing):
    global state, motor_controller, leftPID, rightPID, left_wheel_movement_complete, right_wheel_movement_complete
    global position
    print("TURNING LEFT")
    motor_controller.resetEncoders()
    angle = bearing # turn the difference in angle
    set_point_left = -(primary_robot.axle_length / 2.0) * radians(angle)  # S = R * Theta
    set_point_right = (primary_robot.axle_length / 2.0) * radians(angle)
    leftPID.setPoint(set_point_left)
    rightPID.setPoint(set_point_right)
    left_wheel_movement_complete = False
    right_wheel_movement_complete = False
    state = States.GO_TO_GOAL
    print("SETPOINT LEFT= {}, SET_POINT_RIGHT={}, current={},{}".format(set_point_left, set_point_right,
                                                                        motor_controller.get_left_distance(),
                                                                        motor_controller.get_right_distance()))
def doAction(actionAsString): # find what action to do and talk to the node
    global arm_controller_pub, state, arm_state, motor_controller, leftPID, rightPID, left_wheel_movement_complete, right_wheel_movement_complete, cylinders
    global lcd_pub
    #rospy.loginfo("DO ACTION" + actionAsString)
    if actionAsString == "grabber_arm_up":
        arm_controller_pub.publish("up")
        rospy.loginfo("ARM UP ACTION SENT")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "grabber_arm_down":
        arm_controller_pub.publish("down")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "grabber_arm_open":
        arm_controller_pub.publish("open")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "grabber_arm_open_fully":
        arm_controller_pub.publish("open_fully")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "grabber_arm_close":
        arm_controller_pub.publish("close")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "slowly_forward":
        state = States.SLOWLY_FORWARD
    elif actionAsString == "slowly_backward":
        # Drive slowly backward -10cm
        motor_controller.resetEncoders()
        motor_controller.set_left_speed(145)
        motor_controller.set_right_speed(145)
        time.sleep(2)
        #drive_forward(-300)
        state = States.CHOOSE_TASK
    elif actionAsString == "grabber_arm_default":
        #arm_controller_pub.pub("default")
        #arm_state = ArmState.BUSY
        #state = States.WAITING_FOR_ACTION_FINISH
        pass
    elif actionAsString == "holder_release":
        arm_controller_pub.publish("holder_release")
        arm_state = ArmState.BUSY
        time.sleep(2)
        state = States.WAITING_FOR_ACTION_FINISH
        pass
    elif actionAsString == "holder_default":
        arm_controller_pub.publish("holder_default")
        arm_state = ArmState.BUSY
        time.sleep(2)
        arm_controller_pub.publish("open_fully")
        cylinders-=1
        state = States.WAITING_FOR_ACTION_FINISH
        pass
    elif actionAsString == "rotate_cylinder":
        # Check the light and rotate 1 cylinder until correct colour
        pass

def action_complete(data): # call back when arm action is complete
    global global_actions, arm_state, state
    rospy.loginfo("Action Complete")
    arm_state = ArmState.COMPLETE 


def updatePos(data):
    global position
    position = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.orientation.x]
    print("Updated position to: ",position)

def bump_detected(msg):
    global state, cylinders
    print(msg.data)
    if msg.data == BumperType.GRABBER_BUMPER and cylinders!=4: # we hit a cylinder, grab it
        state = States.STOP_MOTORS
        cylinders+=1
        print("Bumb Detected")
    elif msg.data == "moon_base_bump" and state ==States.SLOWLY_FORWARD: # we hit the moonbase bump
        state = States.STOP_MOTORS
    elif msg.data == BumperType.START_BUMPER: # Start main start_stop_program
        state = States.CHOOSE_TASK


def initialize_node():
    global state, global_actions, arm_state, global_path_instructions, set_point_left, set_point_right, leftPID,\
        rightPID, position, lcd_, arm_controller_pub, prev_state, errorL, errorR
    end_goal_position = []
    rospy.init_node("primary_robot_node") # Initialize the ros node
    rate = rospy.Rate(refresh_rate) # refresh rate
    # Publisherr
    holder_controller_pub = rospy.Publisher("holder", String, queue_size=10)
    lcd_pub = rospy.Publisher("lcd_node", String, queue_size=10)
    # Subscribers
    rospy.Subscriber("arm_action_complete", String, action_complete)
    rospy.Subscriber("bumper_topic", String, bump_detected)
    #Subscribing to ros localization
    rospy.Subscriber("odomtery/filtered", Odometry, updatePos)

    # left and right wheel PID
    left_wheel_movement_complete = False # Whether the left wheel has moved the set distance
    right_wheel_movement_complete = False # Whether the right wheel has moved the set distance

    averagedError = 0
    while not rospy.is_shutdown():
        """
        This is the main state machine, add new states in the StateMachine class.
        """
        if not prev_state == state:
            #lcd_pub.publish("STATE:," + state) 
            print(state)
            prev_state = state 
        if state == States.WAIT_FOR_START_BUTTON:
            # do nothing basically, waiting for someone to pull start button
            print("Waiting to start")
            if not prev_state is state:
                lcd_pub.publish("START SIDE-> B," + state)
        elif state == States.CHOOSE_TASK:
            if not len(global_path_instructions) == 0: # We need to do some moving
                move_action = global_path_instructions[0] # Get first movement command in the list
                if move_action[0] == "drive" and move_action[1] > 0: # We have to drive straight forwards
                    print("DRIVINNG {} CM".format(move_action[1]))
                    set_point_left = move_action[1]
                    set_point_right = move_action[1]
                    #drive_forward(move_action[1])
                    state = States.DRIVE_FORWARD
                    motor_controller.resetEncoders()
                elif move_action[0] == "drive" and move_action[1] < 0:
                    print("DRIVINNG {} CM".format(move_action[1]))
                    set_point_left = move_action[1]
                    set_point_right = move_action[1]
                    state = States.DRIVE_BACKWARD
                    motor_controller.resetEncoders()
                elif move_action[0] == "rotate": # We need  to turn some angle
                    motor_controller.resetEncoders()
                    set_point_left = (primary_robot.axle_length / 2.0) * radians(move_action[1])
                    set_point_right = -1 * (primary_robot.axle_length / 2.0) * radians(move_action[1])
                    print("MOVE STATE, LEFT={}, right={}".format(set_point_left, set_point_right))
                    state = States.TURN_LEFT
            else:
                if not len(global_actions) < 2: # then are actions to do
                    del global_actions[0] #delete previous action
                    rospy.loginfo("THERE ARE " + str(len(global_actions)) + " LEFT")
                    action  = global_actions[0] # Choose first TASK
                    action_type = action[0]
                    if action_type == ActionTypes.GO_TO: # We need to move somewhere
                        end_goal_position = action[0:] # Set the end goal position ["moonbase", "yellow", "A"]
                        motor_controller.resetEncoders()
                        state = States.GO_TO_GOAL
                    if action_type == "drive" and action[1] > 0: # We have to drive straight forwards
                        print("DRIVINNG {} CM".format(action[1]))
                        set_point_left = action[1]
                        set_point_right = action[1]
                        #drive_forward(move_action[1])
                        state = States.DRIVE_FORWARD
                        motor_controller.resetEncoders()
                    elif action_type == "drive" and action[1] < 0:
                        print("DRIVINNG {} CM".format(action[1]))
                        set_point_left = action[1]
                        set_point_right = action[1]
                        state = States.DRIVE_BACKWARD
                        motor_controller.resetEncoders()
                    elif action_type == "rotate": # We need  to turn some angle
                        motor_controller.resetEncoders()
                        set_point_left = (primary_robot.axle_length / 2.0) * radians(action[1])
                        set_point_right = -1 * (primary_robot.axle_length / 2.0) * radians(action[1])
                        print("MOVE STATE, LEFT={}, right={}".format(set_point_left, set_point_right))
                        state = States.TURN_LEFT
                    elif action_type == ActionTypes.ACTION: # We need to do action such as open grabber
                        doAction(action[1])
                    else:
                        pass
                        #print("no instructions to do")

        #this code was at the same level as state machine and move_action is defined inside global_path_instruction branch, so I commented it out
        # if move_action[0] == "action":
        #     action = move_action[1]
        #     doAction(action[1])
        #     if move_action[0] == "rotate": # We need  to turn some angle
        #             motor_controller.resetEncoders()
        #             set_point_left = (primary_robot.axle_length / 2.0) * radians(move_action[1])
        #             set_point_right = -1 * (primary_robot.axle_length / 2.0) * radians(move_action[1])
        #             print("MOVE STATE, LEFT={}, right={}".format(set_point_left, set_point_right))
        #             state = States.TURN_LEFT
        #     else:
        #         if not len(global_actions) < 2: # then are actions to do
        #             del global_actions[0] #delete previous action
        #             rospy.loginfo("THERE ARE " + str(len(global_actions)) + " LEFT")
        #             action  = global_actions[0] # Choose first TASK
        #             action_type = action[0]
        #             if action_type == ActionTypes.GO_TO: # We need to move somewhere
        #                 end_goal_position = action[0:] # Set the end goal position ["moonbase", "yellow", "A"]
        #                 motor_controller.resetEncoders()
        #                 state = States.GO_TO_GOAL
        #             elif action_type == ActionTypes.ACTION: # We need to do action such as open grabber
        #                 doAction(action[1])
        #         else:
        #             pass
        #             #print("no instructions to do")
        elif state == States.GET_PATH: # return path using A* or pre coded route
            print("State = GET PATH")
            global_path_instructions = my_map.get_instructions(position, end_goal_position) # Get the path
            print(global_path_instructions)
            state = States.CHOOSE_TASK # Switch states
        elif state == States.WAITING_FOR_ACTION_FINISH:
            #waiting for something like grabber to finish picking up
            global arm_state
            print(arm_state)
            if arm_state == ArmState.BUSY:
                pass # do nothing
            elif arm_state == ArmState.COMPLETE: # has finished picking or dropping something
                state = States.CHOOSE_TASK # Go do the next task
            pass
        elif state == States.GO_TO_GOAL: # use PID to drive to target distance or turning
            #print("Left Dist = {}, right Dist={}, Left error={}, right error={}".format(motor_controller.get_left_distance(), motor_controller.get_right_distance(), leftPID.getError(), rightPID.getError()))
            left_speed = leftPID.update(motor_controller.get_left_distance())
            right_speed = rightPID.update(motor_controller.get_right_distance())
            print("LEFT DIST={},RIGHT DIST={}".format(motor_controller.get_left_distance(), motor_controller.get_right_distance()))
            if abs(leftPID.getError()) > 0.015:
                motor_controller.set_left_speed(225 - left_speed)
            else:
                left_wheel_movement_complete = True

            if abs(rightPID.getError()) > 0.015:
                motor_controller.set_right_speed(225 - right_speed)
            else:
                right_wheel_movement_complete = True

            if left_wheel_movement_complete and right_wheel_movement_complete:
                state = States.STOP_MOTORS
        elif state == States.DRIVE_FORWARD:

            if motor_controller.get_left_distance() < set_point_left:

                motor_controller.set_acceleration(2)
                motor_controller.set_left_speed(100)
                motor_controller.set_right_speed(100)

            else:
                
                #leftBefore, rightBefore = motor_controller.get_left_distance(), motor_controller.get_right_distance()
                motor_controller.stop()
                motor_controller.set_acceleration(8)
                time.sleep(0.3)
                #print("leftStopDistance= {}, rightStopSitance={}".format(motor_controller.get_left_distance()-leftBefore, motor_controller.get_right_distance()-rightBefore))
                errorR = set_point_right -  motor_controller.get_right_distance()
                errorL = set_point_left - motor_controller.get_left_distance()


                motor_controller.set_acceleration(1)
                motor_controller.resetEncoders()
                #state = States.STOP_MOTORS
                state = States.ERROR_CORRECT

        elif state == States.DRIVE_BACKWARD:
            if motor_controller.get_left_distance() > set_point_left:
                    motor_controller.set_acceleration(2)
                    motor_controller.set_left_speed(148)
                    motor_controller.set_right_speed(148)

            else:
                motor_controller.set_acceleration(8)
                motor_controller.stop()
                time.sleep(0.3)
                print("actualLEFT = {}, actualright={}, set_point_left={}".format(motor_controller.get_left_distance(), motor_controller.get_right_distance(), set_point_left))
                # averagedDistance = (motor_controller.get_left_distance() + motor_controller.get_right_distance()) / 2
                # averagedError = averagedDistance - set_point_left
                errorR = set_point_right -  motor_controller.get_right_distance()
                errorL = set_point_left - motor_controller.get_left_distance()
                # print("averagederror={}".format(averagedError))
                motor_controller.set_acceleration(1)
                motor_controller.resetEncoders()
                state = States.ERROR_CORRECT


        elif state ==  States.TURN_LEFT:
            print("Set point left={}, left_distance={}".format(set_point_left, motor_controller.get_left_distance()))
            if motor_controller.get_left_distance() < set_point_left:
                    motor_controller.set_acceleration(5)
                    motor_controller.set_left_speed(110)
                    motor_controller.set_right_speed(146)
            else:
                motor_controller.set_acceleration(8)
                motor_controller.stop()
                time.sleep(0.3)
                print("LEFT={}, right={},actualLeft={}. actualRight={}".format(set_point_left, set_point_right, motor_controller.get_left_distance(), motor_controller.get_right_distance()))
                # errorR = motor_controller.get_right_distance() - set_point_right
                # errorL = motor_controller.get_left_distance() - set_point_left

                errorR = set_point_right -  motor_controller.get_right_distance()
                errorL = set_point_left - motor_controller.get_left_distance()

                print("LEFT DIST={}, errorL={}".format(motor_controller.get_left_distance(), errorL))
                motor_controller.resetEncoders()
                motor_controller.set_acceleration(1)
                state = States.ERROR_CORRECT

        elif state == States.ERROR_CORRECT:
            forward_correct()
            # print("ERROR CORRRECRT ENTER")
            # if (errorL > 0):
            #     print("ERROR CORRECT, LEFT DIST={}".format(motor_controller.get_left_distance()))
            #     if -(motor_controller.get_left_distance()) < errorL:
            #         motor_controller.set_acceleration(2)
            #         motor_controller.set_left_speed(138)
            #         motor_controller.set_right_speed(118)
            #         print("distanceaftercorrection={}, initialerror={}".format(-(motor_controller.get_left_distance()), errorL))
            #     else:
            #         state = States.STOP_MOTORS
            #         errorL = 0

            # if (errorL < 0):
            #     print("undershoot, left_distance={}".format(motor_controller.get_left_distance()))
            #     if abs(motor_controller.get_left_distance()) < abs(errorL):
            #         print("go forwards")
            #         motor_controller.set_left_speed(120)
            #         motor_controller.set_right_speed(140)
            #     else:
            #         state=States.STOP_MOTORS
            #         errorL = 0

            # if (averagedError > 0):
            #     print("averagedError={}".format(averagedError))
            #     if -(motor_controller.get_left_distance()) < averagedError:
            #         motor_controller.set_left_speed(140)
            #         motor_controller.set_right_speed(140)

            #     else:
            #         state= States.STOP_MOTORS
            #         averagedError = 0

            # if (averagedError < 0):
            #     print("averageCorrect")
            #     if abs(motor_controller.get_left_distance()) <  abs(averagedError):
            #         print("go back")
            #         motor_controller.set_left_speed(120)
            #         motor_controller.set_right_speed(120)
            #     else:
            #         state= States.STOP_MOTORS
            #         averagedError = 0



        elif state == States.TURN_RIGHT:
            if motor_controller.get_left_distance() > set_point_left:
                    motor_controller.set_left_speed(138)
                    motor_controller.set_right_speed(118)
            else:
                 print("LEFT={}, right={},actualLeft={}. actualRight={}".format(set_point_left, set_point_right, motor_controller.get_left_distance(), motor_controller.get_right_distance()))
                 state = States.ERROR_CORRECT

        elif state == States.STOP_MOTORS:  # stop the motors
            motor_controller.set_acceleration(8)
            motor_controller.stop()
            time.sleep(0.3)
            print("STOP MOTOR ")
            if len(global_path_instructions):
                del global_path_instructions[0]
            state = States.CHOOSE_TASK
        elif state == States.SLOWLY_FORWARD:
            motor_controller.resetEncoders()
            # Set a small distance at which to travel slowly for grabbing cylinder
            state = States.GO_TO_GOAL_SLOWLY
        elif state == States.GO_TO_GOAL_SLOWLY: # Go to goal at a slow speed
                motor_controller.set_left_speed(120)
                motor_controller.set_right_speed(120)
        elif state == States.EMERGENCY_STOP:
            motor_controller.stop()
        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
