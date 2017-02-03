import low_level_primary


# -- Primary --

# Drives to crater from current position and aligns with the crater
def go_to_crater(crater):
    pass

# Performs different moves depending on the crater to pick up from
# Assumes the robot is near the crater
def pick_up_balls(crater):
    pass
""" Example: 
    start_drum()
    if crater is "A":
        drive(150)
    elif crater is "B":
        drive(100)
    elif crater is "C"
        drive(100)
        rotate(90)
        drive(100)
        rotate(90)
    stop_drum() 
"""

# Drives to the seasaw
def go_to_seasaw():
    pass

# Unloads the balls in to the cargo bay net
# Assumes: the robot is in front of the seasaw position
def put_balls_in_net():
    pass
""" Example: 
    if blue:
        front_seasaw_arm_down()
        drive(700)
        front_seasaw_arm_up()
        open_basket()
        wait() # or vibrate/unblock balls if stuck
        close_basket()
        # no need to lower the seasaw arm since the seasaw will be down already
        drive(-700)
    else:
        back_seasaw_arm_down()
        drive(-700)
        back_seasaw_arm_up()
        open_basket()
        wait() # or vibrate/unblock balls if stuck
        close_basket()
        # no need to lower the seasaw arm since the seasaw will be down already
        drive(700)
"""