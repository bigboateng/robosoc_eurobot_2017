from StateMachine import StateMachine
from CalibrateSensors import calibrate_sensors
from SelectTask import select_task
from GoToBase import go_to_base
from AStarPath import a_star_path
from AvoidObstacle import avoid_obstacle
from GoToTarget import go_to_target

def start_transitions():
    newState = "calibrate_sensors_state"
    return (newState)

def calibrate_sensors_state_transitions(action):

    action = calibrate_sensors
    
    if action == "select_task":
        newState = "select_task_state"
    if action == "end":
        newState = "end_state"
    else:
        newState = "error_state"
    return (newState, action)

def select_task_state_transitions(action):

    action = select_task

    if action == "no_tasks":
        newState = "go_to_base_state"
    if action == "move_to_target":
        newState = "a_star_path_state"
    else:
        newState = "error_state"
    return (newState, action)

def go_to_base_state_transitions(action):

    go_to_base

    if action == "end":
        newState = "end_state"
    else:
        newState = "error_state"
    return (newState, action)

def a_star_path_state_transitions(action, intial, final):

    a_star_path(intial, final)

    if action == "path_found":
        newState = "go_to_target_state"
    else:
        newState = "error_state"
    return (newState, action)

def go_to_target_state_transitions(action, coordinates):

    go_to_target(coordinates)

    if action == "reached_target":
        newState = "run_main_task_state"
    if action == "obstacle_on_path":
        newState = "avoid_obstacle_state"
    else:
        newState = "error_state"
    return (newState, action)

def avoid_obstacle_state_transitions(action):

    avoid_obstacle

    if action == "avoided":
        newState = "a_star_path
    else:
        newState = "error_state"
    return (newState, action)

def 

def align_moonbase_state_transitions(action):

    # Put code to align with the moonbase here
    if action == "drive":
        newState = "drive_state"
    else:
        newState = "error_state"
    return (newState, action)


if __name__== "__main__":
    m = StateMachine()
    m.add_state("Start", start_transitions)
    m.add_state("calibrate_sensors_state", calibrate_sensors_state_transitions)
    m.add_state("select_task_state", select_task_state_transitions)
    m.add_state("a_star_path_state", a_star_path_state_transitions)
    m.add_state("go_to_target", go_to_target_state_transitions)
    m.add_state("avoid_obstacle", avoid_obstacle_state_transitions)    
    m.add_state("error_state", None, end_state=1)
    m.add_state("end_state", None, end_state=1)
    m.set_start("Start")
