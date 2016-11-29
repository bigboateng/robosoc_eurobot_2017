from StateMachine import StateMachine

def start_transitions():
    newState = "Drive"
    return (newState)

def drive_state_transitions(action):

    # Put Drive Code here
    if action == "rotate":
        newState = "rotate_state"
    if action == "drop_block":
        newState = "drop_block_state"
    if action == "pick_up_block":
        newState = "pick_up_block_state"
    if action == "align_moonbase":
        newState = "align_moonbase_state"
    if action == "drive":
        newState = "drive_state"
    if action == "end":
        newState = "end_state"
    else:
        newState = "error_state"
    return (newState, action)

def rotate_state_transitions(action):

    # Put rotation code here (- rotate for reverse)
    if action == "rotate":
        newState = "rotate_state"
    if action == "drive":
        newState = "drive_state"
    else:
        newState = "error_state"
    return (newState, action)

def align_moonbase_state_transitions(action):

    # Put code to align with the moonbase here
    if action == "drive":
        newState = "drive_state"
    else:
        newState = "error_state"
    return (newState, action)

def pick_up_block_state_transitions(action):

    # Code to pick up block
    if action == "drive":
        newState = "drive_state"
    else:
        newState = "error_state"
    return (newState, action)

def drop_block_state_transitions(action):

    # Code to drop block
    if action == "drive":
        newState = "drive_state"
    else:
        newState = "error_state"
    return (newState, action)

if __name__== "__main__":
    m = StateMachine()
    m.add_state("Start", start_transitions)
    m.add_state("drive_state", drive_state_transitions)
    m.add_state("rotate_state", rotate_state_transitions
    m.add_state("align_moonbase_state", align_moonbase_state_transitions)
    m.add_state("pick_up_block_state", pick_up_block_state_transitions)
    m.add_state("drop_block_state", drop_block_state_transitions)
    m.add_state("error_state", None, end_state=1)
    m.add_state("end_state", None, end_state=1)
    m.set_start("Start")
