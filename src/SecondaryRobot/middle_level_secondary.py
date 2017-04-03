# -- Secondary --

drop_cylinders = []

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
def pick_up_cylinder(needRotating):
    if needRotating:
        drop_cylinders.extend([[["action","rotation_arm_down"],
                               ["action","rotate_cylinder"],
                               ["action","rotation_arm_up"],
                               ["action", "place_cylinder"]]])
    else:
        drop_cylinders.append([["action", "place_cylinder"]])
    return [["action", "grabber_arm_down"], ["action", "grabber_arm_open"],
            ["action", "slowly_forward"], ["action", "grabber_arm_close"],
            ["action", "slowly_backward"], ["action", "grabber_arm_up"],
            ["action", "grabber_arm_open"], ["action", "grabber_arm_default"]]

def pick_up_last_cylinder(needRotating):
    if needRotating:
        drop_cylinders.extend([[["action","rotation_arm_down"],
                               ["action","rotate_cylinder"],
                               ["action","rotation_arm_up"],
                               ["action", "place_cylinder"]]])
    else:
        drop_cylinders.append([["action", "place_cylinder"]])
    return [["action", "grabber_arm_down"], ["action", "grabber_arm_open"],
            ["action", "slowly_forward"], ["action", "grabber_arm_close"],
            ["action", "slowly_backward"], ["action","grabber_arm_up"]]

def drop_cylinder():
    return drop_cylinders.pop(0)

# Drive to the specifies moonbase and align with the slot
def go_to_moonbase(moonbase, isYellow):
    returnList = []
    if isYellow:
        if moonbase == "A":
            returnList.append(["go_to", "moonbase", "yellow", "A"])
	else:
            returnList.append(["go_to", "moonbase", "yellow", "B"])
    else:
        if moonbase == "A":
            returnList.append(["go_to", "moonbase", "yellow", "A"])
	else:
            returnList.append(["go_to", "moonbase", "yellow", "B"])
    return returnList
