# -- Primary --

# Drives to crater from current position and aligns with the crater
def go_to_crater_small(isYellow):

    # Go to the correct position
    if(isYellow == True):
        returnList.append(["go_to(600, 2800)"])
    else:
	    returnList.append(["go_to(200, 600)"])
	return returnList

# Drives to the large crater a
def go_to_large_crater_a(isYellow):
	
	# Go to correct position
	if(isYellow == True):
        returnList.append(["go_to(1490, 3000)"])
    else:
	    returnList.append(["go_to(1490, 0)"])
	return returnList

# Drives to the large crater b
def go_to_large_crater_b(isYellow):
	
	# Go to correct position
	if(isYellow == True):
        returnList.append(["go_to(1639, 2640)"])
    else:
	    returnList.append(["go_to(1639, 360)"])
	return returnList

# Drives to the large crater c
def go_to_large_crater_c(isYellow):
	
	# Go to correct position
	if(isYellow == True):
        returnList.append(["go_to(2000, 2490)"])
    else:
	    returnList.append(["go_to(2000, 510)"])
	return returnList
	

# Performs different moves depending on the crater to pick up from
# Assumes the robot is near the crater
def pick_up_balls(crater):
	returnList.append(["start_drum()"])
    if crater is "A":
        returnList.append(["drive(150)"])
    elif crater is "B":
        returnList.append(["drive(100)"])
    elif crater is "C"
        returnList.append(["drive(100)", "rotate(180)", "drive(100)"])
    returnList.append(["stop_drum()"])
	return returnList

# Drives to the seasaw
def go_to_seasaw(isYellow):
	if isYellow == true:
		returnList.append(["go_to(150, 1930)"])
	else:
		returnList.append(["go_to(150, 1070)"])
    pass

# Go through seasaw
def go_through_seasaw(isYellow):
	if isYellow:
        returnList.append(["front_seasaw_arm_down()", "drive(700)", "front_seasaw_arm_up()"])
    else:
        returnList.append(["back_seasaw_arm_down()", "drive(-700)", "back_seasaw_arm_up()"])
	return returnList
        
# Unloads the balls in to the cargo bay net
# Assumes: the robot is in front of the seasaw position
def put_balls_in_net(isYellow):
	if isYellow:
        returnList.append(["go_to(0, 335)"])
    else:
        returnList.append(["go_to(0, 1665)"])
	
	# Might need some code here if we have a jack
	returnList.append(["open_basket()"])
    return returnList
