class States:
    STOP_MOTORS = "stop_state"
    WAIT_FOR_START_BUTTON = "wait_for_start_button"
    CHOOSE_TASK = "choose_task_state"
    STOP_MAIN_PROGRAM = "stop_state"
    FOLLOW_PATH = "follow_path_state"
    GET_PATH = "get_path_state"
    EMERGENCY_STOP = "emergency_stop"
    WAITING_FOR_ACTION_FINISH = "waiting for action to be done"
    GO_TO_GOAL = "go to goal state"

class ArmState:
    BUSY = "Busy"
    COMPLETE = "Complete"