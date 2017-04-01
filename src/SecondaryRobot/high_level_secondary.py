import middle_level_secondary

# -- Secondary --

# Goes to and picks up n number of rockets from the specified rocket
def get_rocket_cylinders(rocket, n, isYellow):
    returnList = middle_level_secondary.go_to_rocket(rocket, isYellow)
    if n==4:
        for i in range(n-1):
            returnList.extend(middle_level_secondary.pick_up_cylinder())
        returnList.extend(middle_level_secondary.pick_up_last_cylinder())
    else:
        for i in range(n):
            returnList.extend(middle_level_secondary.pick_up_cylinder())
    return returnList

# get last cyclinder i.e. 3 more inside
def get_last_cylinder(cylinder, isYellow):
    returnList = middle_level_secondary.go_to_stray_cylinder(cylinder, isYellow)
    returnList.extend(middle_level_secondary.pick_up_last_cylinder())
    return returnList

# Goes to and picks up a stray cylinder
def get_stray_cylinder(cylinder, isYellow):
    returnList = middle_level_secondary.go_to_stray_cylinder(cylinder, isYellow)
    returnList.extend(middle_level_secondary.pick_up_cylinder())
    return returnList

# Goes to the specified moonbase and places the stored cylinders by going over the slot
def drop_cylinders_in_moonbase(moonbase, no_of_cylinders, isYellow):
    returnList = middle_level_secondary.go_to_moonbase(moonbase, isYellow)
    returnList.extend(middle_level_secondary.place_cylinders_in_slot(no_of_cylinders))
    return returnList

# Goes to the side of the moonbase and rotates each cylinder
def rotate_cylinders_action(moonbase, isYellow):
    returnList = middle_level_secondary.go_to_moonbase(moonbase, isYellow)
    returnList.extend(middle_level_secondary.rotate_cylinders_in_slot())
    return returnList
