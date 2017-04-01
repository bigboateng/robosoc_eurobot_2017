import high_level_secondary

# -- Secondary --

def get_list(isYellow):
    returnList = []
    returnList.extend(high_level_secondary.get_rocket_cylinders("A", 4, isYellow))
    returnList.extend(high_level_secondary.drop_cylinders_in_moonbase("B", 4, isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("A", isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("B", isYellow))
    returnList.extend(high_level_secondary.drop_cylinders_in_moonbase("B", 4, isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("C", isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("D", isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("E", isYellow))
    returnList.extend(high_level_secondary.get_stray_cylinder("F", isYellow))
    returnList.extend(high_level_secondary.drop_cylinders_in_moonbase("A", 4, isYellow))
    returnList.extend(high_level_secondary.rotate_cylinders_action("A", isYellow))
    returnList.extend(high_level_secondary.rotate_cylinders_action("B", isYellow))
    return returnList

print get_list(True)
