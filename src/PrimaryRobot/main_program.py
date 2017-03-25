#!/usr/bin/env python
import rospy
from ..MD25 import MD25
from ..Robot import Robot
from std_msgs.msg import String

# object to hold robot parameters
primary_robot = Robot(axle_length=0.181,wheel_diameter=0.1)
# control motors
motor_controller = MD25(robot=primary_robot)
# String for states
wait_for_start_state = "wait_for_state"
choose_task_state = "choose_task_state"
stop_program_state = "stop_state"

# starting state = wait for start buttin
state = wait_for_start_state

# refresh rate of program
refresh_rate = 50 # 50 Hz


def start_stop_program(msg): # this will state the main state machine
    global state, choose_task_state, stop_program_state
    if msg.data == "start":
        state = choose_task_state # begin program
        rospy.loginfo("Starting program...")
    elif msg.data == "stop":
        state = stop_program_state
        rospy.loginfo("Stoping program...")

def stop_program(data): # 90 seconds, so stop code
    global state, stop_program_state
    state = stop_program_state

def initialize_node():
    global state, stop_program_state
    rospy.init_node("primary_robot_node") # Initialize the ros node
    rate = rospy.Rate(refresh_rate) # refresh rate
    # Publishers

    # Subscribers
    start_stop_program_subscriber = rospy.Subscriber("start_stop_program", String, start_stop_program)
    while not rospy.is_shutdown():
        if state == stop_program_state:  # stop the motors
            motor_controller.stop()
        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass