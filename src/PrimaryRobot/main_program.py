#!/usr/bin/env python
import rospy

from src.PythonAstar.map_loader import MapLoader
from ..MD25 import MD25
from ..Robot import Robot
from std_msgs.msg import String
from ..StateMachine.PrimaryRobotState import States, ArmState
import time
# object to hold robot parameters
primary_robot = Robot(axle_length=0.181,wheel_diameter=0.1)
# control motors
motor_controller = MD25(robot=primary_robot)

# starting state = wait for start buttin
state = States.WAIT_FOR_START_BUTTON

# state of robotic arm
arm_state = ArmState.COMPLETE
# refresh rate of program
refresh_rate = 50 # 50 Hz

#configure A*
my_map_loader = MapLoader(file_path = "./300by200")
my_map, array = my_map_loader.load_map(13)

# Actions to be done
global_actions = [(43,44,56.6)]
global_path_instructions = []


# robots current posision
currentPosX, currentPosY = 0,0


# publishers
arm_controller_pub = None # set it up in initialize_node function
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
    if actionAsString == "armUp":
        arm_controller_pub.pub("up")
        state = States.WAITING_FOR_ACTION_FINISH
        arm_state = ArmState.BUSY

def action_complete(data): # call back when arm action is complete
    global arm_state, state
    arm_state == ArmState.COMPLETE

def initialize_node():
    global state, global_actions, arm_state, global_path_instructions
    end_goal_position = (0,0,0)
    rospy.init_node("primary_robot_node") # Initialize the ros node
    rate = rospy.Rate(refresh_rate) # refresh rate
    # Publishers
    arm_controller_pub = rospy.Publisher("arm_controller", String, queue_size=10)
    # Subscribers
    start_stop_program_subscriber = rospy.Subscriber("start_stop_program", String, start_stop_program)
    arm_action_complete_subscriber = rospy.Subscriber("arm_action_complete", String, action_complete)
    while not rospy.is_shutdown():
        if state == States.WAIT_FOR_START_BUTTON:
            # do nothing basically, waiting for someone to pull start button
            pass
        elif state == States.CHOOSE_TASK:
            if not len(global_actions) == 0: # then are actions to do
                action  = global_actions[0]
                if type(action) == 'tuple' and len(action) == 3: # this is a target goal
                    end_goal_position = action[0]
                    state = States.GET_PATH
                elif type(action) == "str": # then its an action like pick up cylinder
                    doAction(action)
        elif state == States.GET_PATH:
            end_position = global_actions[1]
            global_path_instructions = my_map.get_instructions((currentPosX, currentPosY), (end_position[0],end_position[1]), end_position[2])

        elif state == States.WAITING_FOR_ACTION_FINISH:
            # waiting for something like grabber to finish picking up
            if arm_state == ArmState.BUSY:
                pass # do nothing
            elif arm_state == ArmState.COMPLETE: # has finished picking or dropping something
                state == States.CHOOSE_TASK
        elif state == States.GO_TO_GOAL: # use PID to drive to target distance or turning
            pass
        elif state == States.STOP_MOTORS:  # stop the motors
            motor_controller.stop()
            time.sleep(0.2)
        elif state == States.EMERGENCY_STOP:
            motor_controller.stop()
            # stop drum here

        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
