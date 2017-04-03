import middle_level_secondary

# -- Secondary --

no_of_cylinders_moonbase = 0
no_of_cylinders_carrying = 0
moonbase_order = ["B", "A"]

# Goes to the specified moonbase and places the stored cylinders by going over the slot
def drop_cylinders_in_moonbase(isYellow):
    global no_of_cylinders_carrying
    global no_of_cylinders_moonbase
    returnList = []
    if no_of_cylinders_moonbase > 6:
        returnList.extend(middle_level_secondary.go_to_moonbase(moonbase_order[1], isYellow))
    else:
        returnList.extend(middle_level_secondary.go_to_moonbase(moonbase_order[0], isYellow))
    while no_of_cylinders_carrying != 0:
        no_of_cylinders_carrying = no_of_cylinders_carrying -1
        if no_of_cylinders_moonbase == 6:
            returnList.extend(middle_level_secondary.go_to_moonbase(moonbase_order[1], isYellow))
        returnList.extend(middle_level_secondary.drop_cylinder())
        no_of_cylinders_moonbase = no_of_cylinders_moonbase + 1
    return returnList

# Goes to and picks up n number of rockets from the specified rocket
def get_rocket_cylinders(rocket, n, isYellow):
    global no_of_cylinders_carrying
    returnList = middle_level_secondary.go_to_rocket(rocket, isYellow)
    for i in range(n):
        if no_of_cylinders_carrying == 3:
            if rocket == "A":
                returnList.extend(middle_level_secondary.pick_up_last_cylinder(True))
            else:
                returnList.extend(middle_level_secondary.pick_up_last_cylinder(False))
        else:
            if rocket == "A":
                returnList.extend(middle_level_secondary.pick_up_cylinder(True))
            else:
                returnList.extend(middle_level_secondary.pick_up_cylinder(False))
        no_of_cylinders_carrying = no_of_cylinders_carrying + 1
        if no_of_cylinders_carrying == 4:
            returnList.extend(drop_cylinders_in_moonbase(isYellow))

            # Drive back to rocket if more cylinders to collect
            if i != n-1:
                returnList.extend(middle_level_secondary.go_to_rocket(rocket, isYellow))
    return returnList
            
# Goes to and picks up a stray cylinder
def get_stray_cylinder(cylinder, isYellow):
    global no_of_cylinders_carrying
    returnList = []
    returnList = middle_level_secondary.go_to_stray_cylinder(cylinder, isYellow)
    if no_of_cylinders_carrying == 3:
        if cylinder == "B" or cylinder == "D" or cylinder == "E":
            returnList.extend(middle_level_secondary.pick_up_last_cylinder(True))
        else:
            returnList.extend(middle_level_secondary.pick_up_last_cylinder(False))
    else:
        if cylinder == "B" or cylinder == "D" or cylinder == "E":
            returnList.extend(middle_level_secondary.pick_up_cylinder(True))
        else:
            returnList.extend(middle_level_secondary.pick_up_cylinder(False))
    no_of_cylinders_carrying = no_of_cylinders_carrying + 1
    if no_of_cylinders_carrying == 4:
        returnList.extend(drop_cylinders_in_moonbase(isYellow))
    return returnList