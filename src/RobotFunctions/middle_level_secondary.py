import low_level_secondary


# -- Secondary --

# Drive to the specified rocket cylinders
def go_to_rocket(rocket):
    pass

# Picks up a single cylinder from the rocket
# Assumes the grabber arm is down and the robot is in front of rocket
def pick_up_rocket_cylinder():
    pass
""" Example:
    drive(150)
    grab_cylinder()
    drive(-150)
    grabber_arm_up()
    store_cylinder()
    grabber_arm_down()
"""

# Goes to the specifies cylinder
def go_to_stray_cylinder(cylinder):
    pass

# Picks up a stray cylinder (i.e. not one in the rocket)
# Assumes the grabber arm is down and the robot is in front of cylinder
def pick_up_stray_cylinder():
    pass
""" Example
    drive(150)
    grab_cylinder()
    # Does not drive backwards.
    grabber_arm_up()
    store_cylinder()
    grabber_arm_down()
"""

# Drive to the specifies moonbase and align with the slot
def go_to_moonbase(moonbase):
    pass

# Places all cylinders in the moonbase slot
# Assumes over the slot
def place_cylinders_in_slot():
    pass
""" Example:
    drive(120*no_of_cylinders)
    while no_of_cylinders > 0:
        place_cylinder()
        drive(-120)
        no_of_cylinders -= 1
"""

# Drive to the specified moonbase and align with the side of the moonbase
def go_to_moonbase_side(moonbase):
    pass

# Rotates the cylinder until colour is facing the right way up.
# Assumes the robot is on the side of the moonbase
# Assumes rotation arm is up, in natural position
def rotate_cylinders_in_slot():
    pass
""" Example: 
    drive(50)
    for i in range(6): # 6 is the max number of cylinders that fit in the middle slot
        rotation_arm_down()
        rotate_cylinder()
        rotation_arm_up()
        if i != 5:
            drive(100)
    drive(-500)
"""

