import middle_level_primary

# -- Primary --

# Goes to the specified crater and picks up balls
def get_small_crater_balls_a(isYellow):
    go_to_crater(isYellow)
    pick_up_balls("A")

# Go through the seasaw
def go_though_seasaw(isYellow):
    returnList = []
    if isYellow:
        returnList.append(["front_seasaw_arm_down()", "drive(700)", "front_seasaw_arm_up()"])
    else:
        returnList.append(["back_seasaw_arm_down()", "drive(-700)", "back_seasaw_arm_up()"])
    return returnList

# Goes to the cargo bay in the starting area to place the balls in the net 
#   and then go back through the seasaw
def drop_off_balls(isYellwow):
    go_to_seasaw(isYellow)
    go_through_seasaw(isYellow)
    put_balls_in_net()
    go_through_seasaw(not (isYellow))
	
# Get the large crater balls
def get_large_crater_balls():
    go_to_large_crater_a(isYellow)
    pick_up_balls("C")
    go_to_large_crater_b(isYellow)
    pick_up_balls("C")
    go_to_large_crater_c(isYellow)
    pick_up_balls("C")
	
