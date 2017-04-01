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
def pick_up_cylinder():
    return [["action", "grabber_arm_down"], ["action", "grabber_arm_open"],
            ["action", "slowly_forward"], ["action", "grabber_arm_close"],
            ["action", "grabber_arm_up"], ["action", "grabber_arm_open"],
            ["action", "grabber_arm_default"]]

def pick_up_last_cylinder():
    return [["action", "grabber_arm_down"], ["action", "grabber_arm_open"],
            ["action", "slowly_forward"], ["action", "grabber_arm_close"],
            ["action","grabber_arm_up"]]

# Drive to the specifies moonbase and align with the slot
def go_to_moonbase(moonbase, isYellow):
    returnList = []
    if isYellow:
        if moonbase == "A":
            returnList.append(["go_to", "moonbase", "yellow", "A"])
	elif moonbase == "B":
            returnList.append(["go_to", "moonbase", "yellow", "B"])
	else:
            returnList.append(["go_to", "moonbase", "yellow", "C"])
    else:
        if moonbase == "A":
            returnList.append(["go_to", "moonbase", "yellow", "A"])
	elif moonbase == "B":
            returnList.append(["go_to", "moonbase", "yellow", "B"])
	else:
            returnList.append(["go_to", "moonbase", "yellow", "C"])
    return returnList

# Places all cylinders in the moonbase slot
# Assumes over the slot
def place_cylinders_in_slot(no_of_cylinders):
    returnList = []
    while no_of_cylinders > 0:
        returnList.append(["action", "place_cylinder"])
        no_of_cylinders -= 1
    return returnList

# Rotates the cylinder until colour is facing the right way up.
# Assumes the robot is on the side of the moonbase
# Assumes rotation arm is up, in natural position
def rotate_cylinders_in_slot():
    returnList = [["drive", "72"]]
    for i in range(6): # 6 is the max number of cylinders that fit in the middle slot
        returnList.extend([["action","rotation_arm_down"], ["action","rotate_cylinder"],
                           ["action","rotation_arm_up"]])
        if i != 5:
            returnList.append(["drive", "-12"])
    return returnList
