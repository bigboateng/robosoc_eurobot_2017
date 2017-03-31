# -- Secondary --

# Drive to the specified rocket cylinders
def go_to_rocket(rocket, isYellow):
    returnList = []
    if isYellow:
        if rocket == "A":
            returnList.append(["go_to", "rocket", "yellow", "A"])
        else:
            returnList.append(["go_to", "rocket", "yellow", "B"])
    else:
        if rocket == "A":
            returnList.append(["go_to", "rocket", "blue", "A"])
        else:
            returnList.append(["go_to", "rocket", "blue", "B"])
    return returnList

# Picks up a single cylinder from the rocket
# ASSUMPTION: grab_cylinder includes: arm_down, arm_open, drive to cylinder, arm_close
def pick_up_rocket_cylinder():
    return ["grab_cylinder", "grabber_arm_up", "store_cylinder", "grabber_arm_down"])

# Goes to the specifies cylinder
def go_to_stray_cylinder(cylinder, isYellow):
    returnList = []
    if isYellow:
        if cylinder == "A":
            returnList.append(["go_to", "cylinder", "yellow", "A"])
	elif cylinder == "B":
            returnList.append(["go_to", "cylinder", "yellow", "B"])
	elif cylinder == "C":
            returnList.append(["go_to", "cylinder", "yellow", "C"])
	elif cylinder == "D":
            returnList.append(["go_to", "cylinder", "yellow", "D"])
	elif cylinder == "E":
            returnList.append(["go_to", "cylinder", "yellow", "E"])
	elif cylinder == "F":
            returnList.append(["go_to", "cylinder", "yellow", "F"])
    else:
        if cylinder == "A":
            returnList.append(["go_to", "cylinder", "blue", "A"])
	elif cylinder == "B":
            returnList.append(["go_to", "cylinder", "blue", "B"])
	elif cylinder == "C":
            returnList.append(["go_to", "cylinder", "blue", "C"])
	elif cylinder == "D":
            returnList.append(["go_to", "cylinder", "blue", "D"])
	elif cylinder == "E":
            returnList.append(["go_to", "cylinder", "blue", "E"])
	elif cylinder == "F":
            returnList.append(["go_to", "cylinder", "blue", "F"])
    return returnList
	
# Picks up a stray cylinder (i.e. not one in the rocket)
# Assumes the grabber arm is down and the robot is in front of cylinder
def pick_up_stray_cylinder():
	returnList.append(["drive(150)", "grab_cylinder()", "grabber_arm_up()", "store_cylinder()", "grabber_arm_down()"])

# Drive to the specifies moonbase and align with the slot
def go_to_moonbase(isYellow, moonbase):
	if(isYellow)
		if(moonbase == "A"):
			returnList.append(["goTo(1400,900)"])
		else
			returnList.append(["goTo(1200,1500)"])
	else
		if(moonbase == "A"):
			returnList.append(["goTo(1400,900)"])
		else
			returnList.append(["goTo(1200,1500)"])
    pass

# Places all cylinders in the moonbase slot
# Assumes over the slot
def place_cylinders_in_slot(no_of_cylinders):
    returnList.append(["drive(" +120*no_of_cylinders + ")"])
    while no_of_cylinders > 0:
        returnList.append(["place_cylinder()", "drive(-120)"])
        no_of_cylinders -= 1

# Drive to the specified moonbase and align with the side of the moonbase
def go_to_moonbase_side(moonbase):
    pass

# Rotates the cylinder until colour is facing the right way up.
# Assumes the robot is on the side of the moonbase
# Assumes rotation arm is up, in natural position
def rotate_cylinders_in_slot():
    returnList.append(["drive(50)"])
    for i in range(6): # 6 is the max number of cylinders that fit in the middle slot
        returnList.append(["rotation_arm_down()", "rotate_cylinder()", "rotation_arm_up()"])
        if i != 5:
            returnList.append(["drive(100)"])
    drive(-500)

