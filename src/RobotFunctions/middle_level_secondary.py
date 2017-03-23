# -- Secondary --

# Drive to the specified rocket cylinders
def go_to_rocket(rocket, isYellow):
    if isYellow:
		if rocket == "A":
			returnList.append(["goTo(3000, 1350)"])
		else:
			returnList.append(["goTo(0, 1850)"])
	else:
		if rocket == "A":
			returnList.append(["goTo(1350, 0)"])
		else:
			returnList.append(["goTo(0, 1150)"])

# Picks up a single cylinder from the rocket
# Assumes the grabber arm is down and the robot is in front of rocket
def pick_up_rocket_cylinder():
    returnList.append(["drive(150)", "grab_cylinder()", "drive(-150)", "grabber_arm_up()", "store_cylinder()", "grabber_arm_down()"])

# Goes to the specifies cylinder
def go_to_stray_cylinder(cylinder, isYellow):
	if isYellow:
		if cylinder == "A":
			returnList.append(["goTo(600, 2800)"])
		elif cylinder == "B":
			returnList.append(["goTo(1100, 2500)"])
		elif cylinder == "C":
			returnList.append(["goTo(1850, 2200)"])
		elif cylinder == "D":
			returnList.append(["goTo(1400, 2100)"])
		elif cylinder == "E":
			returnList.append(["goTo(600, 2000)"])
		elif cylinder == "F":
			returnList.append(["goTo(200, 2050)"])
	else:
		if cylinder == "A":
			returnList.append(["goTo(200, 600)"])
		elif cylinder == "B":
			returnList.append(["goTo(1100, 500)"])
		elif cylinder == "C":
			returnList.append(["goTo(1850, 800)"])
		elif cylinder == "D":
			returnList.append(["goTo(1400, 900)"])
		elif cylinder == "E":
			returnList.append(["goTo(600, 1000)"])
		elif cylinder == "F":
			returnList.append(["goTo(200, 950)"])
	
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

