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
motor_controller = MD25(address=0x59, robot=primary_robot)
motor_controller.resetEncoders()
motor_controller.set_acceleration(2)

# starting state = wait for start button
#state = States.WAIT_FOR_START_BUTTON
state = States.CHOOSE_TASK
#state = States.CHOOSE_TASK
prev_state = state

# number of cylinders that robot carry
cylinders = 0

# current position - updated by subscribing to ros
position = [0.0,0.0,0.0]

# state of robotic arm
arm_state = ArmState.COMPLETE
# refresh rate of program
refresh_rate = 80 # 50 Hz

#configure A*
my_map_loader = None #MapLoader(file_path = "./PythonAstar/300by200")
my_map, array = None, None #my_map_loader.load_map(0)

# Actions to be done
#global_actions = [["action", "del"], ["action", "grabber_arm_close"], ["action", "grabber_arm_down"], ["action", "grabber_arm_open"]]#, ["action", "grabber_arm_close"], ["action", "grabber_arm_up"], ["action", "grabber_arm_open"]]
global_actions = [["action", "del"],["action", "grabber_arm_close"], ["action", "grabber_arm_down"], ["action", "grabber_arm_open"],["action", "slowly_forward"], ["action", "grabber_arm_close"], ["action", "grabber_arm_up"], ["action", "grabber_arm_open"]]
#global_path_instructions = [("drive", 0.4),("drive", 0.35) , ("bearing", -3.14)]
global_actions = [[],["action", "slowly_backward"]]
global_path_instructions = []

# robots current position
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

def start_stop_program(msg): # this will state the main state machine
    global state
    if msg.data == "start":
        state = States.CHOOSE_TASK # begin program
        rospy.loginfo("Starting program...")
    elif msg.data == "stop": # 90 seconds, stop program
        state = States.EMERGENCY_STOP
        rospy.loginfo("Stoping program...")


def drive_forward(dist):
    global state, motor_controller, leftPID, rightPID, left_wheel_movement_complete, right_wheel_movement_complete
    motor_controller.resetEncoders()
    set_point_left = dist / 100  # Set the left wheel PID, convert the distance to meters
    set_point_right = dist / 100  # Set the left wheel PID, convert the distance to meters
    leftPID.setPoint(set_point_left)  # Setting the left PID set point
    rightPID.setPoint(set_point_right)  # Setting the right PID set point
    left_wheel_movement_complete = False  # Reset the left wheel
    right_wheel_movement_complete = False  # Reset the right wheel
    print("SETPOINT LEFT= {}, SET_POINT_RIGHT={}, current={},{}".format(set_point_left,
                                                                        set_point_right,
                                                                        motor_controller.get_left_distance(),
                                                                        motor_controller.get_right_distance()))
    state = States.GO_TO_GOAL  # Now distances are set, let GO_TO_GOAL do the movements


def turn(bearing):
    global state, motor_controller, leftPID, rightPID, left_wheel_movement_complete, right_wheel_movement_complete
    global position
    print("TURNING LEFT")
    motor_controller.resetEncoders()
    angle = position[2] - bearing # turn the difference in angle
    set_point_left = -(primary_robot.axle_length / 2) * radians(angle)  # S = R * Theta
    set_point_right = (primary_robot.axle_length / 2) * radians(angle)
    leftPID.setPoint(set_point_left)
    rightPID.setPoint(set_point_right)
    left_wheel_movement_complete = False
    right_wheel_movement_complete = False
    state = States.GO_TO_GOAL
    print("SETPOINT LEFT= {}, SET_POINT_RIGHT={}, current={},{}".format(set_point_left, set_point_right,
                                                                        motor_controller.get_left_distance(),
                                                                        motor_controller.get_right_distance()))
def doAction(actionAsString): # find what action to do and talk to the node
    global arm_controller_pub, state, arm_state, motor_controller, leftPID, rightPID, left_wheel_movement_complete, right_wheel_movement_complete
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
    elif actionAsString == "grabber_arm_close":
        arm_controller_pub.publish("close")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "slowly_forward":
        state = States.SLOWLY_FORWARD
    elif actionAsString == "slowly_backward":
        # Drive slowly backward -10cm
        motor_controller.resetEncoders()
        motor_controller.set_left_speed(135)
        motor_controller.set_right_speed(135)
        time.sleep(4)
        state = States.CHOOSE_TASK
    elif actionAsString == "grabber_arm_default":
        arm_controller_pub.pub("default")
        arm_state = ArmState.BUSY
        state = States.WAITING_FOR_ACTION_FINISH
    elif actionAsString == "place_cylinder":
        # Drop the cylinder into moonbase and move it forward
        # Check cylinder can be placed and put in starting area if cannot
        # todo(Boateng)
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
    global state
    print(msg.data)
    if msg.data == BumperType.GRABBER_BUMPER: # we hit a cylinder, grab it
        state = States.STOP_MOTORS
        print("Bumb Detected")
    elif msg.data == "moon_base_bump": # we hit the moonbase bump
        pass
    elif msg.data == BumperType.START_BUMPER: # Start main start_stop_program
        state = States.CHOOSE_TASK

def initialize_node():
    global state, global_actions, arm_state, global_path_instructions, set_point_left, set_point_right, leftPID,\
        rightPID, position, lcd_, arm_controller_pub, prev_state
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
                if move_action[0] == "drive": # We have to drive straight forwards
                   drive_forward(move_action[1])
                elif move_action[0] == "bearing": # We need  to turn some angle
                    turn(move_action[1])
            else:
                if not len(global_actions) == 1: # then are actions to do
                    del global_actions[0] #delete previous action
                    rospy.loginfo("THERE ARE " + str(len(global_actions)) + " LEFT")
                    action  = global_actions[0] # Choose first TASK
                    action_type = action[0]
                    if action_type == ActionTypes.GO_TO: # We need to move somewhere
                        end_goal_position = action[0:] # Set the end goal position ["moonbase", "yellow", "A"]
                        state = States.GO_TO_GOAL
                    elif action_type == ActionTypes.ACTION: # We need to do action such as open grabber
                        doAction(action[1])
                else:
                    pass
                    #print("no instructions to do")
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
            if abs(leftPID.getError()) > 0.01:
                motor_controller.set_left_speed(225 - left_speed)
            else:
                left_wheel_movement_complete = True

            if abs(rightPID.getError()) > 0.01:
                motor_controller.set_right_speed(225 - right_speed)
            else:
                right_wheel_movement_complete = True
            if left_wheel_movement_complete and right_wheel_movement_complete:
                state = States.STOP_MOTORS
        elif state == States.STOP_MOTORS:  # stop the motors
            motor_controller.stop()
            time.sleep(0.5)
            print("STOP MOTOR ")
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
